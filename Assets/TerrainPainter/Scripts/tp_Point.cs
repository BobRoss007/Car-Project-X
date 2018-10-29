////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_Point.cs
//      Author:             HOEKKII
//      
//      Description:        A integer x and y
//      
////////////////////////////////////////////////////////////////////////////

using UnityEngine;

namespace TerrainPainter
{
    public struct Point
    {
        public int x, y;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        public Point(int x, int y)
        {
            this.x = x;
            this.y = y;
        }
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        public void Set(int x, int y)
        {
            this.x = x;
            this.y = y;
        }

        /// <summary>
        /// Point(0, 0)
        /// </summary>
        public static Point zero { get { return new Point(0, 0); } }
        /// <summary>
        /// Point(1, 1)
        /// </summary>
        public static Point one { get { return new Point(1, 1); } }

        /// <summary>
        /// Get the min of both
        /// 
        /// Example:
        ///     a = (1, 6)
        ///     b = (4, 5)
        /// 
        ///     return (1, 5)
        /// 
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static Point Min(Point a, Point b)
        {
            return new Point(a.x > b.x ? b.x : a.x, a.y > b.y ? b.y : a.y);
        }
        /// <summary>
        /// Get the max of both
        /// 
        /// Example:
        ///     a = (1, 6)
        ///     b = (4, 5)
        /// 
        ///     return (4, 6)
        /// 
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static Point Max(Point a, Point b)
        {
            return new Point(a.x < b.x ? b.x : a.x, a.y < b.y ? b.y : a.y);
        }

        /// <summary>
        /// Converts to readable string
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return "[" + x.ToString() + ", " + y.ToString() + "]";
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            return base.Equals(obj);
        }
        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        #region Operators
        public static Point operator +(Point a, Point b)
        {
            return new Point(a.x + b.x, a.y + b.y);
        }
        public static Point operator -(Point a)
        {
            return new Point(-a.x, -a.y);
        }
        public static Point operator -(Point a, Point b)
        {
            return new Point(a.x - b.x, a.y - b.y);
        }
        public static Point operator *(Point a, int b)
        {
            return new Point(a.x * b, a.y * b);
        }
        public static Point operator *(int b, Point a)
        {
            return new Point(a.x * b, a.y * b);
        }
        public static Point operator /(Point a, int b)
        {
            return new Point(a.x / b, a.y / b);
        }
        public static bool operator ==(Point a, Point b)
        {
            return a.x == b.x && a.y == b.y;
        }
        public static bool operator !=(Point a, Point b)
        {
            return a.x != b.x && a.y != b.y;
        }

        public Point Swap()
        {
            return new Point(y, x);
        }

        public static implicit operator Vector2(Point a)
        {
            return new Vector2((float)a.x, (float)a.y);
        }
        public static explicit operator Point(Vector2 a)
        {
            return new Point(Mathf.RoundToInt(a.x), Mathf.RoundToInt(a.y));
        }
        #endregion
    }
}
