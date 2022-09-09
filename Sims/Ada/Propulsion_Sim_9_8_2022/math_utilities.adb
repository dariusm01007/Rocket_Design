-- Like the .cpp file corresponding to the .h file in C++
-- this file will have all of the function definitions

with Ada.Numerics.Elementary_Functions;
use  Ada.Numerics.Elementary_Functions;

package body math_utilities is

   -- ============= Function Definitions =============

   -- ---------------------------------------------------
   --   Function    : deg2rad()
   -- 
   --   Description : Converts angles from degrees to radians
   --
   --   Example call: angle := deg2rad(25.0);
   -- ---------------------------------------------------

   function deg2rad(angle : in float) return float is
   begin
      return angle * (Ada.Numerics.Pi/180.0);
   end deg2rad;
   

   -- ---------------------------------------------------
   --   Function    : rad2deg()
   -- 
   --   Description : Converts angles from radians to degrees
   --
   --   Example call: angle := rad2deg(0.1222);
   -- ---------------------------------------------------

   function rad2deg(angle: in float) return float is
   begin
      return angle * (180.0/Ada.Numerics.Pi);
   end rad2deg;
   
   
   -- ---------------------------------------------------
   --   Function    : atan2()
   -- 
   --   Description : Computes 4 quadrant inverse tangent
   --
   --   Example call: angle := atan2(y,x)
   -- ---------------------------------------------------

   function atan2(y, x : in float) return float is
      -- storing the answer in a var to be returned
      output : float;

      -- Laziness
      zero : float := 0.0;
      pi : float := Ada.Numerics.Pi;

   begin

      -- First Quadrant (+,+)
      if x > zero and y > zero then
         output := Arctan(y,x);

      -- Second Quadrant (-,+)
      elsif x < zero and y >= zero then
         output := Arctan(y,x) + pi;

      -- Third Quadrant (-,-)
      elsif x < zero and y < zero then
         output := Arctan(y,x) - pi;

      -- Fourth Quadrant (+,-)
      elsif x > zero and y <= zero then
         output := Arctan(y,x);

      -- 90 deg (straight up)
      elsif x = zero and y > zero then
         output := pi * 0.5;

      -- 270 deg (straight down)
      elsif x = zero and y < zero then
         output := -pi * 0.5;

      -- Technically undefined
      elsif x = zero and y = zero then
         output := zero;
      end if;


      return output;

   end atan2;

   -- ---------------------------------------------------
   --   Function    : dynamicPressure()
   -- 
   --   Description : Calculates dynamic pressure using an 
   --                 exponential air density approximation
   --
   --   Example call: Q := dynamicPressure(22300.0, 45.8);
   -- ---------------------------------------------------
   
   function dynamicPressure(alt, V : in float) return float is
      rho : float;
   begin

      if alt < 9144.0 then
         rho := 1.2255708354*exp(-alt/9144.0);
      else
         rho := 1.75228799*exp(-alt/6705.6);
      end if;
      
      -- Density in kg/m^3

      return 0.5*rho*(V**2);
   end dynamicPressure;
   

   -- ---------------------------------------------------
   --   Function    : accel2G()
   -- 
   --   Description : Converts acceleration from m/s^2 to G
   --
   --   Example call: accel := accel2G(9.81);
   -- ---------------------------------------------------

   function accel2G(accel : in float) return float is
   begin
      return accel/9.81;
   end accel2G;
   

end math_utilities;
