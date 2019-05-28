using System.Runtime.CompilerServices;


namespace Demos.Port.CollisionGroups
{
    /// <summary>
    /// Bit masks which control whether different members of a group of objects can collide with each other.
    /// </summary>
    public struct CollisionGroup
    {
        /// <summary>
        /// A mask of 16 bits, each set bit representing a collision group that an object can interact with.
        /// </summary>
        public uint CollidableSubgroups;


        public uint DetectionSubgroups;
        /// <summary>
        /// Id of the owner of the object. Objects belonging to different groups always collide.
        /// </summary>
        public uint GroupId;

        /// <summary>
        /// Initializes a collision filter that collides with everything in the group.
        /// </summary>
        /// <param name="groupId">Id of the group that this filter operates within.</param>
        public CollisionGroup(uint groupId)
        {
            GroupId = groupId;
            CollidableSubgroups = uint.MaxValue;
            DetectionSubgroups = uint.MaxValue;
        }

        public void EnableCollision(uint subgroupId)
        {
            CollidableSubgroups |= subgroupId;
        }

        public void DisableCollision(uint subgroupId)
        {
            CollidableSubgroups &= ~subgroupId;
        }

        public void EnableDetection(uint subgroupId)
        {
            DetectionSubgroups |= subgroupId;
        }

        public void DisableDetection(uint subgroupId)
        {
            DetectionSubgroups &= ~subgroupId;
        }

        /// <summary>
        /// Enables the collision.
        /// </summary>
        ///
        /// <param name="a">    First filter to test. </param>
        /// <param name="b">    Second filter to test. </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void EnableCollision(in CollisionGroup a, in CollisionGroup b)
        {
            a.EnableCollision(b.GroupId);
            b.EnableCollision(a.GroupId);
        }

        /// <summary>
        /// Disables the collision.
        /// </summary>
        ///
        /// <param name="a">    First filter to test. </param>
        /// <param name="b">    Second filter to test. </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void DisableCollision(in CollisionGroup a, in CollisionGroup b)
        {
            a.DisableCollision(b.GroupId);
            b.DisableCollision(a.GroupId);
        }


        /// <summary>
        /// Enables the collision.
        /// </summary>
        ///
        /// <param name="a">    First filter to test. </param>
        /// <param name="b">    Second filter to test. </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void EnableDetection(in CollisionGroup a, in CollisionGroup b)
        {
            a.EnableDetection(b.GroupId);
            b.EnableDetection(a.GroupId);
        }

        /// <summary>
        /// Disables the collision.
        /// </summary>
        ///
        /// <param name="a">    First filter to test. </param>
        /// <param name="b">    Second filter to test. </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void DisableDetection(in CollisionGroup a, in CollisionGroup b)
        {
            a.DisableDetection(b.GroupId);
            b.DisableDetection(a.GroupId);
        }

        /// <summary>
        /// Checks if the filters can collide by checking if b's membership can be collided by a's collidable groups.
        /// </summary>
        /// <param name="a">First filter to test.</param>
        /// <param name="b">Second filter to test.</param>
        /// <returns>True if the filters can collide, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool AllowCollision(in CollisionGroup a, in CollisionGroup b)
        {
            return  (a.GroupId & b.CollidableSubgroups) != 0;
        }

    }
}