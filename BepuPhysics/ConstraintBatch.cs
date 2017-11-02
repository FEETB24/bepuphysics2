﻿using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Constraints;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    /// <summary>
    /// Contains a set of type batches whose constraints share no body references.
    /// </summary>
    public struct ConstraintBatch
    {
        //Note that both active and inactive constraint batches share the same data layout.
        //This means we have a type id->index mapping in inactive islands. 
        //Reasoning:
        //The type id->index mapping is required because the solver's handle->constraint indirection stores a type id. If it stored a batch-specific type *index*, 
        //then all mappings associated with a type batch's constraints would have to be updated when a type batch changes slots due to a removal.
        //Given that there could be hundreds or thousands of such changes required, we instead rely on this last-second remapping.

        //However, an inactive island will never undergo removals under normal conditions, and they can only happen at all by direct user request.
        //In other words, the risk of moving type batches in inactive islands is pretty much irrelevant. In fact, you could instead store a direct handle->type *index* mapping
        //pretty safely, even if it meant either not moving type batches when one becomes empty or just brute force updating the mapping associated with every constraint in the type batch.

        //The cost of storing this extra data is not completely trivial. Assuming an average of 128 bytes per type id->index mapping, consider what happens
        //when you have 65536 inactive islands: ~10 megabytes of wasted mapping data.

        //Right now, we make no attempt to split the storage layout and bite the bullet on the waste, because:
        //1) While it does require some memory, 10 megabytes is fairly trivial in terms of *capacity* for any platform where you're going to have a simulation with 65536 inactive islands.
        //2) The memory used by a bunch of different islands isn't really concerning from a bandwidth perspective, because by nature, it is not being accessed every frame (nor all at once).
        //3) Avoiding this waste would require splitting the storage representation and/or complicating the handle->constraint mapping.
        //TODO: So, perhaps one day we'll consider changing this, but for now it's just not worth it.

        //That said, we DO store each active constraint batch's BatchReferencedHandles separately from the ConstraintBatch type.
        //Two reasons for the different choice:
        //1) The handles memory will often end up being an order of magnitude bigger. We're not talking about 10MB for 65536 inactive islands here, but rather 100-400MB and up.
        //2) Storing the referenced handles separately in the solver doesn't really change anything. We just have to pass them as parameters here and there; no significant complication.

        public Buffer<int> TypeIndexToTypeBatchIndex;
        public QuickList<TypeBatchData, Buffer<TypeBatchData>> TypeBatches;

        public ConstraintBatch(BufferPool pool, int initialTypeCountEstimate = 32)
            : this()
        {
            ResizeTypeMap(pool, initialTypeCountEstimate);
            QuickList<TypeBatchData, Buffer<TypeBatchData>>.Create(pool.SpecializeFor<TypeBatchData>(), initialTypeCountEstimate, out TypeBatches);
        }

        void ResizeTypeMap(BufferPool pool, int newSize)
        {
            var oldLength = TypeIndexToTypeBatchIndex.Length;
            pool.SpecializeFor<int>().Resize(ref TypeIndexToTypeBatchIndex, newSize, oldLength);
            for (int i = oldLength; i < TypeIndexToTypeBatchIndex.Length; ++i)
            {
                TypeIndexToTypeBatchIndex[i] = -1;
            }
        }

        [Conditional("DEBUG")]
        void ValidateTypeBatchMappings()
        {
            for (int i = 0; i < TypeIndexToTypeBatchIndex.Length; ++i)
            {
                var index = TypeIndexToTypeBatchIndex[i];
                if (index >= 0)
                {
                    Debug.Assert(index < TypeBatches.Count);
                    Debug.Assert(TypeBatches[index].TypeId == i);
                }
            }
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                Debug.Assert(TypeIndexToTypeBatchIndex[TypeBatches[i].TypeId] == i);
            }
        }

        /// <summary>
        /// Gets a type batch in the batch matching the given type id.
        /// Requires that there exists at least one constraint in the type batch.
        /// </summary>
        /// <param name="typeId">Id of the TypeBatch's type to retrieve.</param>
        /// <returns>TypeBatch instance associated with the given type.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TypeBatchData GetTypeBatch(int typeId)
        {
            ValidateTypeBatchMappings();
            var typeBatchIndex = TypeIndexToTypeBatchIndex[typeId];
            return ref TypeBatches[typeBatchIndex];
        }

        ref TypeBatchData CreateNewTypeBatch(int typeId, TypeProcessor typeProcessor, int initialCapacity, BufferPool pool)
        {
            var newIndex = TypeBatches.Count;
            TypeBatches.EnsureCapacity(TypeBatches.Count + 1, pool.SpecializeFor<TypeBatchData>());
            TypeIndexToTypeBatchIndex[typeId] = newIndex;
            ref var typeBatch = ref TypeBatches.AllocateUnsafely();
            typeProcessor.Initialize(ref typeBatch, initialCapacity, pool);
            return ref typeBatch;
        }


        internal ref TypeBatchData GetOrCreateTypeBatch(int typeId, TypeProcessor typeProcessor, int initialCapacity, BufferPool pool)
        {
            if (typeId >= TypeIndexToTypeBatchIndex.Length)
            {
                //While we only request a capacity one slot larger, buffer pools always return a power of 2, so this isn't going to cause tons of unnecessary resizing.
                ResizeTypeMap(pool, typeId + 1);
                return ref CreateNewTypeBatch(typeId, typeProcessor, initialCapacity, pool);
            }
            else
            {
                var typeBatchIndex = TypeIndexToTypeBatchIndex[typeId];
                if (typeBatchIndex == -1)
                {
                    return ref CreateNewTypeBatch(typeId, typeProcessor, initialCapacity, pool);
                }
                else
                {
                    return ref TypeBatches[typeBatchIndex];
                }
            }
        }

        public unsafe void Allocate(int handle, ref int constraintBodyHandles, int bodyCount, ref BatchReferencedHandles existingHandles, Bodies bodies,
            int typeId, TypeProcessor typeProcessor, int initialCapacity, BufferPool pool, out ConstraintReference reference)
        {
            //Add all the constraint's body handles to the batch we found (or created) to block future references to the same bodies.
            //Also, convert the handle into a memory index. Constraints store a direct memory reference for performance reasons.
            var bodyIndices = stackalloc int[bodyCount];
            for (int j = 0; j < bodyCount; ++j)
            {
                var bodyHandle = Unsafe.Add(ref constraintBodyHandles, j);
                existingHandles.Add(bodyHandle, pool);
                bodyIndices[j] = bodies.HandleToIndex[bodyHandle];
            }
            ref var typeBatch = ref GetOrCreateTypeBatch(typeId, typeProcessor, initialCapacity, pool);
            reference = new ConstraintReference(ref typeBatch, typeProcessor.Allocate(ref typeBatch, handle, bodyIndices, pool));
            //TODO: We could adjust the typeBatchAllocation capacities in response to the allocated index.
            //If it exceeds the current capacity, we could ensure the new size is still included.
            //The idea here would be to avoid resizes later by ensuring that the historically encountered size is always used to initialize.
            //This isn't necessarily beneficial, though- often, higher indexed batches will contain smaller numbers of constraints, so allocating a huge number
            //of constraints into them is very low value. You may want to be a little more clever about the heuristic. Either way, only bother with this once there is 
            //evidence that typebatch resizes are ever a concern. This will require frame spike analysis, not merely average timings.
            //(While resizes will definitely occur, remember that it only really matters for *new* type batches- 
            //and it is rare that a new type batch will be created that actually needs to be enormous.)
        }


        unsafe struct BodyHandleRemover : IForEach<int>
        {
            public Bodies Bodies;
            //TODO: When blittable rolls around, we should be able to de-void this.
            public void* Handles;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public BodyHandleRemover(Bodies bodies, ref BatchReferencedHandles handles)
            {
                Bodies = bodies;
                Handles = Unsafe.AsPointer(ref handles);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int bodyIndex)
            {
                Unsafe.AsRef<BatchReferencedHandles>(Handles).Remove(Bodies.IndexToHandle[bodyIndex]);
            }
        }

        //Note that we have split the constraint batch removal for the sake of reuse by the multithreaded constraint remover.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveTypeBatchIfEmpty(ref TypeBatchData typeBatch, int typeBatchIndexToRemove, BufferPool pool)
        {
            if (typeBatch.ConstraintCount == 0)
            {
                var constraintTypeId = typeBatch.TypeId;
                TypeIndexToTypeBatchIndex[constraintTypeId] = -1;
                TypeBatches.FastRemoveAt(typeBatchIndexToRemove);
                if (typeBatchIndexToRemove < TypeBatches.Count)
                {
                    //If we swapped anything into the removed slot, we should update the type index to type batch mapping.
                    TypeIndexToTypeBatchIndex[TypeBatches[typeBatchIndexToRemove].TypeId] = typeBatchIndexToRemove;
                }
                typeBatch.Dispose(pool);
            }
            ValidateTypeBatchMappings();
        }

        public unsafe void RemoveWithHandles(int constraintTypeId, int indexInTypeBatch, ref BatchReferencedHandles handles, Solver solver)
        {
            Debug.Assert(TypeIndexToTypeBatchIndex[constraintTypeId] >= 0, "Type index must actually exist within this batch.");

            var typeBatchIndex = TypeIndexToTypeBatchIndex[constraintTypeId];
            var handleRemover = new BodyHandleRemover(solver.bodies, ref handles);
            ref var typeBatch = ref TypeBatches[typeBatchIndex];
            solver.TypeProcessors[constraintTypeId].EnumerateConnectedBodyIndices(ref typeBatch, indexInTypeBatch, ref handleRemover);
            Remove(ref typeBatch, typeBatchIndex, indexInTypeBatch, solver.TypeProcessors[constraintTypeId], ref solver.HandleToConstraint, solver.bufferPool);

        }

        public unsafe void Remove(int constraintTypeId, int indexInTypeBatch, Solver solver)
        {
            Debug.Assert(TypeIndexToTypeBatchIndex[constraintTypeId] >= 0, "Type index must actually exist within this batch.");

            var typeBatchIndex = TypeIndexToTypeBatchIndex[constraintTypeId];
            ref var typeBatch = ref TypeBatches[typeBatchIndex];
            Remove(ref TypeBatches[typeBatchIndex], typeBatchIndex, indexInTypeBatch, solver.TypeProcessors[constraintTypeId], ref solver.HandleToConstraint, solver.bufferPool);
        }

        unsafe void Remove(ref TypeBatchData typeBatch, int typeBatchIndex, int indexInTypeBatch, TypeProcessor typeProcessor, ref Buffer<ConstraintLocation> handleToConstraint, BufferPool pool)
        {
            typeProcessor.Remove(ref typeBatch, indexInTypeBatch, ref handleToConstraint);
            RemoveTypeBatchIfEmpty(ref typeBatch, typeBatchIndex, pool);
        }

        public void Clear(BufferPool pool)
        {
            for (int typeBatchIndex = 0; typeBatchIndex < TypeBatches.Count; ++typeBatchIndex)
            {
                TypeBatches[typeBatchIndex].Dispose(pool);
            }
            //Since there are no more type batches, the mapping must be cleared out.
            for (int typeId = 0; typeId < TypeIndexToTypeBatchIndex.Length; ++typeId)
            {
                TypeIndexToTypeBatchIndex[typeId] = -1;
            }
            TypeBatches.Clear();
        }

        public void Resize(Solver solver, int bodiesCount, int constraintTypeCount)
        {
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                ref var typeBatch = ref TypeBatches[i];
                solver.TypeProcessors[TypeBatches[i].TypeId].Resize(ref TypeBatches[i], Math.Max(typeBatch.ConstraintCount, solver.GetMinimumCapacityForType(typeBatch.TypeId)), solver.bufferPool);
            }
            //For now this is mostly just for rehydration. Note that it's actually an EnsureCapacity. For simplicity, we just don't permit the compaction of the type batch arrays.
            if (TypeIndexToTypeBatchIndex.Length < constraintTypeCount)
            {
                ResizeTypeMap(solver.bufferPool, constraintTypeCount);
                if (!TypeBatches.Span.Allocated)
                    QuickList<TypeBatchData, Buffer<TypeBatchData>>.Create(solver.bufferPool.SpecializeFor<TypeBatchData>(), constraintTypeCount, out TypeBatches);
                else
                    TypeBatches.Resize(constraintTypeCount, solver.bufferPool.SpecializeFor<TypeBatchData>());
            }
        }
        /// <summary>
        /// Disposes the unmanaged resources used by the batch and drops all pooled managed resources.
        /// </summary>
        /// <remarks>Calling Resize will make the batch usable again after disposal.</remarks>
        public void Dispose(BufferPool pool)
        {
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                TypeBatches[i].Dispose(pool);
            }
            TypeBatches.Clear();
            pool.SpecializeFor<int>().Return(ref TypeIndexToTypeBatchIndex);
            TypeIndexToTypeBatchIndex = new Buffer<int>();
            TypeBatches.Dispose(pool.SpecializeFor<TypeBatchData>());
            TypeBatches = new QuickList<TypeBatchData, Buffer<TypeBatchData>>();
        }
    }
}