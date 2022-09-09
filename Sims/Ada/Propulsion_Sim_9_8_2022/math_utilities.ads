-- Similar to a C++ header file the package ads file contains 
-- function declarations

package math_utilities is
   
   -- ============= Function Declarations =============
   
   -- ---------------------------------------------------
   --   Function    : deg2rad()
   -- 
   --   Description : Converts angles from degrees to radians
   --
   --   Example call: angle := deg2rad(25.0);
   -- ---------------------------------------------------

   function deg2rad(angle : in float) return float;


   -- ---------------------------------------------------
   --   Function    : rad2deg()
   -- 
   --   Description : Converts angles from radians to degrees
   --
   --   Example call: angle := rad2deg(0.1222);
   -- ---------------------------------------------------
   
   function rad2deg(angle : in float) return float;


   -- ---------------------------------------------------
   --   Function    : atan2()
   -- 
   --   Description : Computes 4 quadrant inverse tangent
   --
   --   Example call: angle := atan2(y,x)
   -- ---------------------------------------------------
   
   function atan2(y, x : in float) return float;


   -- ---------------------------------------------------
   --   Function    : dynamicPressure()
   -- 
   --   Description : Calculates dynamic pressure using an 
   --                 exponential air density approximation
   --
   --   Example call: Q := dynamicPressure(22300.0, 45.8);
   -- ---------------------------------------------------
   
   function dynamicPressure(alt, V : in float) return float;


   -- ---------------------------------------------------
   --   Function    : accel2G()
   -- 
   --   Description : Converts acceleration from m/s^2 to G
   --
   --   Example call: accel := accel2G(9.81);
   -- ---------------------------------------------------
   
   function accel2G(accel : in float) return float;
   
end math_utilities;
