using System;
using System.Runtime.InteropServices;

namespace lib_trilateration {
    [StructLayout(LayoutKind.Sequential)]
    public struct Coord3D {
        public double x;
        public double y;
        public double z;

        public Coord3D(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public override string ToString()
        {
            const string format = "{0:0.000}";
            var formattedX = string.Format(format, x);
            var formattedY = string.Format(format, y);
            var formattedZ = string.Format(format, z);
            return $"{formattedX}, {formattedY}, {formattedZ}";
        }

        public static Coord3D FromString(string In)
        {
            var strValues = In.Split(',');
            if (strValues.Length != 3)
            {
                throw new FormatException("String length is different than 3.");
            }
            return new Coord3D(
                Convert.ToDouble(strValues[0]),
                Convert.ToDouble(strValues[1]),
                Convert.ToDouble(strValues[2])
                );
        }
    }

	public static class Trilateration
	{
        public enum Status
        {
            TRIL_3SPHERES = 3,
            TRIL_4SPHERES = 4,
            ERR_TRIL_CONCENTRIC = -1,
            ERR_TRIL_COLINEAR_2SOLUTIONS = -2,
            ERR_TRIL_SQRTNEGNUMB = -3
        }
        
		public static Status GetLocation(out Coord3D bestSolution, Coord3D[] anchorArray, uint[] distanceArray)
		{
            return (Status) get_location(out bestSolution, anchorArray.Length >= 4, anchorArray, distanceArray);
		}
        
        
		[DllImport("lib_trilateration.dll", EntryPoint = "get_location", SetLastError = true)]
		private static extern int get_location(out Coord3D bestSolution, bool use4ThAnchor, [In, Out] Coord3D[] anchorArray, [In, Out] uint[] distanceArray);
	}
}
