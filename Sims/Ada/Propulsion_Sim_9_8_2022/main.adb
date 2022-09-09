with Ada.Text_IO, Ada.Float_Text_IO;
use Ada.Text_IO, Ada.Float_Text_IO;

with Ada.Numerics.Elementary_Functions;
use  Ada.Numerics.Elementary_Functions;

with math_utilities;
use math_utilities;

-- Parameters from:
-- https://www.apogeerockets.com/Rocket_Motors/Apogee_Medalist/24mm_Motors/Apogee_Medalist_Motor_E6-8_1pk

procedure Main is

   -- Acceleration due to gravity
   g0 : constant float := 9.81; -- [m/s^2]

   -- Mass properties
   m_dry : float := 46.2/1000.0; -- [kg] dry mass
   m_pl  : float := 0.0;         -- [kg] payload mass
   m_p   : float := 22.0/1000.0; -- [kg] propellant mass

   -- Rocket motor properties
   Thrust : float := 16.6;             -- [N] max thrust
   tb     : float := 5.8;              -- [s] burn time
   mdot   : float := m_p/tb;           -- [kg/s] mass flow rate
   Isp    : float := Thrust/(g0*mdot); -- [s] specific impulse

   -- Overall Mass
   m_o : float := m_dry + m_pl + m_p;

   -- Final Mass
   m_f : float := m_o - m_p;

   -- Current Mass
   m   : float := m_o;

   -- Rocket equation
   deltaV : float := Isp*g0*log(m_o/m_f);

   -- Time related parameters
   t  : float := 0.0;  -- [s] start time
   dt : float := 0.01; -- [s] time step

   -- Initial angle
   gamma : float := deg2rad(85.0); -- [rad]

   -- Velocity
   V : float := 0.0; -- [m/s]

   -- States
   x : float := 0.0; -- [m]
   z : float := 0.0; -- [m]

   xdot : float := V*cos(gamma); -- [m/s]
   zdot : float := V*sin(gamma); -- [m/s]


   -- Drag Force
   beta : float := 250.0; -- [N/m^2] ballistic coefficient
   Q    : float := dynamicPressure(z,V);
   Drag : float := (Q*g0)/beta;

   -- Acceleration
   zDD : float := ((Thrust-Drag)/m)*sin(gamma) - g0;

   xDD : float := ((Thrust-Drag)/m)*cos(gamma);

   A   : float;

   -- Creating txt file
   F : File_Type;
   fileName : constant String := "C:\Users\dariu\OneDrive\Desktop\rocketPropulsionSim.txt";


begin
   
   -- Open the file
   Create (F, Out_File, fileName);
   
    while z >= 0.0 loop

      -- Updating Velocity
      zdot := zdot + zDD*dt;

      xdot := xdot + xDD*dt;

      -- Updating Position
      z := z + zdot*dt;

      x := x + xdot*dt;

      -- Calculate new flight path angle
      gamma := atan2(zdot,xdot);

      -- Calculate velocity magnitude
      V := sqrt(zdot**2 + xdot**2);

      -- Calculate total acceleration
      A := sqrt(zDD**2 + xDD**2);

      -- Update the current mass
      m := m - mdot*dt;

      -- Update time
      t := t + dt;
      
      if t >= tb then
         -- No longer burning fuel, therefore no thrust
         Thrust := 0.0;
         mdot   := 0.0;
      end if;   
       
      -- Save the data
      Put(File=> F, Item => t, Fore => 2, Aft =>4, Exp => 0); -- 2 spaces before decimal, 2 spaces after, no exponents
      Put(F,"   ");
      Put(File=> F, Item => z, Fore => 2, Aft =>4, Exp => 0);
      Put(F,"   ");
      Put(File=> F, Item => x, Fore => 2, Aft =>4, Exp => 0);
      Put(F,"   ");
      Put(File=> F, Item => rad2deg(gamma), Fore => 2, Aft =>4, Exp => 0);
      Put(F,"   ");
      Put(File=> F, Item => V, Fore => 2, Aft =>4, Exp => 0);
      Put(F,"   ");
      Put(File=> F, Item => accel2G(A), Fore => 2, Aft =>4, Exp => 0);
      Put(F,"   ");
      Put(File=> F, Item => m, Fore => 2, Aft =>4, Exp => 0);
      New_Line(F);

      -- Update the dynamic pressure
      Q := dynamicPressure(z,V);

      -- Update Drag
      Drag := (Q*g0)/beta;

      -- Compute the acceleration
      zDD := ((Thrust-Drag)/m)*sin(gamma) - g0;

      xDD := ((Thrust-Drag)/m)*cos(gamma);
       
    end loop;
   
   -- Close the file
   close(F);

   null;
   
end Main;
