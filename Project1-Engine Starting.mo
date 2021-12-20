within ;
package Project1
  model Summer
    Modelica.Mechanics.MultiBody.Examples.Loops.Utilities.Cylinder cylinder(
        crankAngleOffset=3.1415926535898)
      annotation (Placement(transformation(extent={{0,68},{20,88}})));
    inner Modelica.Mechanics.MultiBody.World world
      annotation (Placement(transformation(extent={{-86,38},{-66,58}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute bearing(
      useAxisFlange=true,
      n={1,0,0},
      cylinderLength=0.02,
      cylinderDiameter=0.06,
      animation=false)     annotation (Placement(transformation(extent={{-46,64},
              {-26,44}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-44,20})));
    Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_SeriesExcited dcse(
      TaOperational=298.15,
      alpha20a(displayUnit="1/K"),
      Jr=0.07,
      frictionParameters(PRef=2.5, wRef(displayUnit="rev/min")),
        ia(fixed=false))
      annotation (Placement(transformation(extent={{-36,-72},{-16,-52}})));
    Modelica.Electrical.Batteries.BatteryStacks.CellStack cellStack(
      Ns=1,
      Np=1,
      cellData(
        Qnom(displayUnit="A.h") = 216000,
        OCVmax=12.8,
        OCVmin=12,
        Ri=0.01,
        alpha=0.004,
        Idis=2,
        R0=3),
      SOC(fixed=true, start=0.642),
      useHeatPort=true)
                       annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-74,-42})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-74,-94},{-54,-74}})));
    Modelica.Mechanics.Rotational.Components.Damper damper(d=0.78)
      annotation (Placement(transformation(extent={{44,46},{64,66}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixed
      annotation (Placement(transformation(extent={{66,38},{86,58}})));
    Modelica.Mechanics.Rotational.Components.OneWayClutch oneWayClutch
      annotation (Placement(transformation(extent={{24,-52},{44,-32}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(
      J=0.3,
      w(fixed=false),
      a(fixed=false))
      annotation (Placement(transformation(extent={{8,16},{28,36}})));
    Modelica.Blocks.Sources.Constant const(k=1)
      annotation (Placement(transformation(extent={{6,-20},{26,0}})));
    Modelica.Electrical.Analog.Ideal.IdealClosingSwitch switch
      annotation (Placement(transformation(extent={{-36,-40},{-16,-20}})));
    Modelica.Blocks.Sources.BooleanPulse booleanPulse(
      width=2,
      period=100,
      startTime=0)
      annotation (Placement(transformation(extent={{-56,-16},{-38,2}})));
    Modelica.Mechanics.Rotational.Components.Damper damper1(d=0.1)
      annotation (Placement(transformation(extent={{24,-82},{44,-62}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixed1
      annotation (Placement(transformation(extent={{58,-82},{78,-62}})));
    Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=10)
      annotation (Placement(transformation(extent={{66,-22},{86,-2}})));
    Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=298.15)
      annotation (Placement(transformation(extent={{-88,-16},{-72,0}})));
  equation
    connect(cylinder.cylinder_a, world.frame_b) annotation (Line(
        points={{0,88},{-56,88},{-56,48},{-66,48}},
        color={95,95,95},
        thickness=0.5));
    connect(bearing.frame_b, cylinder.crank_a) annotation (Line(
        points={{-26,54},{-6,54},{-6,68},{0,68}},
        color={95,95,95},
        thickness=0.5));
    connect(bearing.frame_a, world.frame_b) annotation (Line(
        points={{-46,54},{-56,54},{-56,48},{-66,48}},
        color={95,95,95},
        thickness=0.5));
    connect(speedSensor.flange, bearing.axis) annotation (Line(points={{-54,20},
            {-58,20},{-58,38},{-36,38},{-36,44}},                  color={0,0,0}));
    connect(damper.flange_b, fixed.flange)
      annotation (Line(points={{64,56},{76,56},{76,48}},color={0,0,0}));
    connect(dcse.pin_an, dcse.pin_ep) annotation (Line(points={{-32,-52},{-32,
            -42},{-40,-42},{-40,-56},{-36,-56}},
                                            color={0,0,255}));
    connect(dcse.pin_en, ground.p)
      annotation (Line(points={{-36,-68},{-64,-68},{-64,-74}}, color={0,0,255}));
    connect(cellStack.n, ground.p) annotation (Line(points={{-84,-42},{-88,-42},
            {-88,-64},{-64,-64},{-64,-74}},color={0,0,255}));
    connect(bearing.axis, damper.flange_a) annotation (Line(points={{-36,44},{
            -36,38},{32,38},{32,56},{44,56}},
                                          color={0,0,0}));
    connect(inertia.flange_a, damper.flange_a) annotation (Line(points={{8,26},{
            2,26},{2,38},{32,38},{32,56},{44,56}},                       color={0,
            0,0}));
    connect(dcse.flange, oneWayClutch.flange_a)
      annotation (Line(points={{-16,-62},{10,-62},{10,-42},{24,-42}},
                                                   color={0,0,0}));
    connect(const.y, oneWayClutch.f_normalized) annotation (Line(points={{27,-10},
            {36,-10},{36,-24},{34,-24},{34,-31}}, color={0,0,127}));
    connect(booleanPulse.y, switch.control) annotation (Line(points={{-37.1,-7},
            {-26,-7},{-26,-18}},                                         color={
            255,0,255}));
    connect(damper1.flange_a, dcse.flange) annotation (Line(points={{24,-72},{
            -6,-72},{-6,-62},{-16,-62}},
                                      color={0,0,0}));
    connect(fixed1.flange, damper1.flange_b)
      annotation (Line(points={{68,-72},{44,-72}},          color={0,0,0}));
    connect(oneWayClutch.flange_b, idealGear.flange_a) annotation (Line(points={{44,-42},
            {60,-42},{60,-12},{66,-12}},           color={0,0,0}));
    connect(idealGear.flange_b, inertia.flange_b) annotation (Line(points={{86,-12},
            {90,-12},{90,26},{28,26}},      color={0,0,0}));
    connect(fixedTemperature.port, cellStack.heatPort) annotation (Line(points=
            {{-72,-8},{-72,-22},{-74,-22},{-74,-32}}, color={191,0,0}));
    connect(cellStack.p, switch.p) annotation (Line(points={{-64,-42},{-42,-42},
            {-42,-30},{-36,-30}}, color={0,0,255}));
    connect(switch.n, dcse.pin_ap) annotation (Line(points={{-16,-30},{-10,-30},
            {-10,-44},{-20,-44},{-20,-52}}, color={0,0,255}));
  end Summer;

  model cylinder_with_pv
    Modelica.Mechanics.MultiBody.Parts.BodyCylinder piston(
      diameter=0.1,
      r={0,0.1,0},
      color={180,180,180},
      animation=false)     annotation (Placement(transformation(
          origin={20,32},
          extent={{10,10},{-10,-10}},
          rotation=270)));
    Modelica.Mechanics.MultiBody.Parts.BodyBox connectingRod(
      widthDirection={1,0,0},
      height=0.06,
      color={0,0,200},
      width=0.02,
      r_shape={0,-0.02,0},
      r={0,0.2,0},
      animation=false)     annotation (Placement(transformation(
          origin={60,-8},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Joints.Revolute b2(
      n={1,0,0},
      cylinderLength=0.02,
      animation=false,
      cylinderDiameter=0.055) annotation (Placement(transformation(extent={{30,2},{
              50,22}})));
    Modelica.Mechanics.MultiBody.Parts.BodyBox crank4(
      height=0.05,
      widthDirection={1,0,0},
      width=0.02,
      r={0,-0.1,0},
      animation=false)
      annotation (Placement(transformation(
          origin={50,-68},
          extent={{10,-10},{-10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Parts.BodyCylinder crank3(
      r_shape={-0.01,0,0},
      length=0.12,
      diameter=0.03,
      r={0.1,0,0},
      color={180,180,180},
      animation=false)     annotation (Placement(transformation(extent={{10.5,
              -58},{30.5,-38}})));
    Modelica.Mechanics.MultiBody.Parts.BodyCylinder crank1(
      diameter=0.05,
      r_shape={-0.01,0,0},
      length=0.12,
      r={0.2 - 0.1,0,0},
      color={180,180,180},
      animation=false)     annotation (Placement(transformation(extent={{-50,-98},
              {-30,-78}})));
    Modelica.Mechanics.MultiBody.Parts.BodyBox crank2(
      height=0.05,
      widthDirection={1,0,0},
      width=0.02,
      r={0,0.1,0},
      animation=false)     annotation (Placement(transformation(
          origin={-10,-68},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Joints.RevolutePlanarLoopConstraint
                                        b1(
      n={1,0,0},
      cylinderLength=0.02,
      animation=false,
      cylinderDiameter=0.055) annotation (Placement(transformation(extent={{30,-18},
              {50,-38}})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation mid(r={0.1/2,0,0},
        animation=false)
                 annotation (Placement(transformation(extent={{0,-38},{20,-18}})));
    Modelica.Mechanics.MultiBody.Joints.Prismatic cylinder(
      useAxisFlange=false,
      animation=false,
      s(fixed=false),
      n={0,-1,0},
      boxWidth=0.02) annotation (Placement(transformation(
          origin={20,62},
          extent={{-10,-10},{10,10}},
          rotation=270)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation mounting(r={0.2,0,0},
        animation=false)
                 annotation (Placement(transformation(extent={{0,92},{20,112}})));
    Modelica.Mechanics.MultiBody.Parts.FixedRotation cylinderInclination(
      r={0.1/2,0,0},
      n_y={0,Modelica.Math.cos(0),Modelica.Math.sin(0)},
      animation=false,
      rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors)
      annotation (Placement(transformation(extent={{-70,32},{-50,52}})));
    Modelica.Mechanics.MultiBody.Parts.FixedRotation crankAngle1(
      n_y={0,Modelica.Math.cos(180),Modelica.Math.sin(180)},
      animation=false,
      rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors)
      annotation (Placement(transformation(extent={{-90,-98},{-70,-78}})));
    Modelica.Mechanics.MultiBody.Parts.FixedRotation crankAngle2(
      n_y={0,Modelica.Math.cos(-180),Modelica.Math.sin(-180)},
      animation=false,
      rotationType=Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors)
      annotation (Placement(transformation(extent={{70,-98},{90,-78}})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation cylinderTop(r={0,0.42,0},
        animation=false) annotation (Placement(transformation(
          origin={-30,62},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a
                       cylinder_a annotation (Placement(transformation(extent={{-116,86},
              {-84,118}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a
                       cylinder_b annotation (Placement(transformation(extent={{84,86},
              {116,118}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a
                       crank_a annotation (Placement(transformation(extent={{-116,
              -114},{-84,-82}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a
                       crank_b annotation (Placement(transformation(extent={{84,-114},
              {116,-82}})));
    ex1.cylPV
          cylPV1(cylClearance=0.005, stroke=0.405)
      annotation (Placement(transformation(extent={{-9,-10},{9,10}},
          rotation=270,
          origin={48,67})));
  equation
    connect(b1.frame_a,mid. frame_b) annotation (Line(
        points={{30,-28},{20,-28}},
        color={95,95,95},
        thickness=0.5));
    connect(connectingRod.frame_a,b1. frame_b) annotation (Line(
        points={{60,-18},{60,-28},{50,-28}},
        color={95,95,95},
        thickness=0.5));
    connect(cylinder.frame_b,piston. frame_b) annotation (Line(
        points={{20,52},{20,42}},
        color={95,95,95},
        thickness=0.5));
    connect(crank1.frame_a,crankAngle1. frame_b)
      annotation (Line(
        points={{-50,-88},{-70,-88}},
        color={95,95,95},
        thickness=0.5));
    connect(b2.frame_a,piston. frame_a) annotation (Line(
        points={{30,12},{20,12},{20,22}},
        color={95,95,95},
        thickness=0.5));
    connect(connectingRod.frame_b,b2. frame_b) annotation (Line(
        points={{60,2},{60,12},{50,12}},
        color={95,95,95},
        thickness=0.5));
    connect(crank4.frame_b,crankAngle2. frame_a) annotation (Line(
        points={{50,-78},{50,-88},{70,-88}},
        color={95,95,95},
        thickness=0.5));
    connect(cylinderInclination.frame_b,cylinderTop. frame_a)
      annotation (Line(
        points={{-50,42},{-30,42},{-30,52}},
        color={95,95,95},
        thickness=0.5));
    connect(crank1.frame_b,crank2. frame_a) annotation (Line(
        points={{-30,-88},{-10,-88},{-10,-78}},
        color={95,95,95},
        thickness=0.5));
    connect(crank3.frame_b,crank4. frame_a) annotation (Line(
        points={{30.5,-48},{50,-48},{50,-58}},
        color={95,95,95},
        thickness=0.5));
    connect(crank3.frame_a,crank2. frame_b) annotation (Line(
        points={{10.5,-48},{-10,-48},{-10,-58}},
        color={95,95,95},
        thickness=0.5));
    connect(crank2.frame_b,mid. frame_a) annotation (Line(
        points={{-10,-58},{-10,-28},{0,-28}},
        color={95,95,95},
        thickness=0.5));
    connect(cylinderTop.frame_b,cylinder. frame_a) annotation (Line(
        points={{-30,72},{-30,82},{20,82},{20,72}},
        color={95,95,95},
        thickness=0.5));
    connect(cylinderInclination.frame_a,cylinder_a)  annotation (Line(
        points={{-70,42},{-80,42},{-80,102},{-100,102}},
        color={95,95,95},
        thickness=0.5));
    connect(mounting.frame_a,cylinder_a)  annotation (Line(
        points={{0,102},{-100,102}},
        color={95,95,95},
        thickness=0.5));
    connect(mounting.frame_b,cylinder_b)  annotation (Line(
        points={{20,102},{100,102}},
        color={95,95,95},
        thickness=0.5));
    connect(crankAngle1.frame_a,crank_a)  annotation (Line(
        points={{-90,-88},{-100,-88},{-100,-98}},
        color={95,95,95},
        thickness=0.5));
    connect(crankAngle2.frame_b,crank_b)  annotation (Line(
        points={{90,-88},{100,-88},{100,-98}},
        color={95,95,95},
        thickness=0.5));
    connect(cylPV1.frame_b, cylinder.frame_b) annotation (Line(
        points={{48,58},{48,52},{20,52}},
        color={95,95,95},
        thickness=0.5));
    connect(cylPV1.frame_a, cylinder.frame_a) annotation (Line(
        points={{48,76},{48,80},{20,80},{20,72}},
        color={95,95,95},
        thickness=0.5));
    annotation ();
  end cylinder_with_pv;

  model discharge_lights
    Modelica.Electrical.Batteries.BatteryStacks.CellStack cellStack(
      Ns=1,
      Np=1,
      cellData(
        Qnom(displayUnit="A.h") = 216000,
        OCVmax=12.8,
        OCVmin=12,
        Ri=0.01,
        alpha=0.004,
        Idis=2,
        R0=3),
      SOC(fixed=true, start=0.75),
      useHeatPort=true)
                       annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-2,2})));
    Modelica.Electrical.Analog.Basic.Resistor resistor(
      R=10,
      alpha=0.0045,
      useHeatPort=true)
      annotation (Placement(transformation(extent={{-6,42},{14,62}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-62,-4},{-42,16}})));
    Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=298.15)
      annotation (Placement(transformation(extent={{-54,38},{-34,58}})));
  equation
    connect(resistor.p, cellStack.n) annotation (Line(points={{-6,52},{-24,52},
            {-24,2},{-12,2}}, color={0,0,255}));
    connect(resistor.n, cellStack.p)
      annotation (Line(points={{14,52},{14,2},{8,2}},        color={0,0,255}));
    connect(ground.p, cellStack.n) annotation (Line(points={{-52,16},{-38,16},{
            -38,28},{-24,28},{-24,2},{-12,2}}, color={0,0,255}));
    connect(fixedTemperature.port, resistor.heatPort) annotation (Line(points={{-34,48},
            {-10,48},{-10,38},{0,38},{0,36},{4,36},{4,42}},
                                                         color={191,0,0}));
    connect(cellStack.heatPort, fixedTemperature.port) annotation (Line(points=
            {{-2,12},{0,12},{0,38},{-10,38},{-10,48},{-34,48}}, color={191,0,0}));
  end discharge_lights;

  model Winter
    Modelica.Mechanics.MultiBody.Examples.Loops.Utilities.Cylinder cylinder(
        crankAngleOffset=3.1415926535898)
      annotation (Placement(transformation(extent={{2,74},{22,94}})));
    inner Modelica.Mechanics.MultiBody.World world
      annotation (Placement(transformation(extent={{-84,44},{-64,64}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute bearing(
      useAxisFlange=true,
      n={1,0,0},
      cylinderLength=0.02,
      cylinderDiameter=0.06,
      animation=false)     annotation (Placement(transformation(extent={{-44,70},
              {-24,50}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-42,26})));
    Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_SeriesExcited dcse(
      TaOperational=263.15,
      alpha20a(displayUnit="1/K"),
      Jr=0.07,
      frictionParameters(PRef=2.5, wRef(displayUnit="rev/min")),
      ia(fixed=false))
      annotation (Placement(transformation(extent={{-34,-66},{-14,-46}})));
    Modelica.Electrical.Batteries.BatteryStacks.CellStack cellStack(
      Ns=1,
      Np=1,
      cellData(
        Qnom(displayUnit="A.h") = 216000,
        OCVmax=12.8,
        OCVmin=12,
        Ri=0.01,
        alpha=0.004,
        Idis=2,
        R0=3),
      SOC(fixed=true, start=0.635),
      useHeatPort=true)
                       annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-72,-36})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-72,-88},{-52,-68}})));
    Modelica.Mechanics.Rotational.Components.Damper damper(d=4.5)
      annotation (Placement(transformation(extent={{46,52},{66,72}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixed
      annotation (Placement(transformation(extent={{68,44},{88,64}})));
    Modelica.Mechanics.Rotational.Components.OneWayClutch oneWayClutch
      annotation (Placement(transformation(extent={{26,-46},{46,-26}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(
      J=0.3,
      w(fixed=false),
      a(fixed=false))
      annotation (Placement(transformation(extent={{10,22},{30,42}})));
    Modelica.Blocks.Sources.Constant const(k=1)
      annotation (Placement(transformation(extent={{8,-14},{28,6}})));
    Modelica.Electrical.Analog.Ideal.IdealClosingSwitch switch
      annotation (Placement(transformation(extent={{-34,-34},{-14,-14}})));
    Modelica.Blocks.Sources.BooleanPulse booleanPulse(
      width=2,
      period=100,
      startTime=0)
      annotation (Placement(transformation(extent={{-54,-10},{-36,8}})));
    Modelica.Mechanics.Rotational.Components.Damper damper1(d=0.6)
      annotation (Placement(transformation(extent={{26,-76},{46,-56}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixed1
      annotation (Placement(transformation(extent={{60,-76},{80,-56}})));
    Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=10)
      annotation (Placement(transformation(extent={{68,-16},{88,4}})));
    Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=263.15)
      annotation (Placement(transformation(extent={{-86,-10},{-70,6}})));
  equation
    connect(cylinder.cylinder_a,world. frame_b) annotation (Line(
        points={{2,94},{-54,94},{-54,54},{-64,54}},
        color={95,95,95},
        thickness=0.5));
    connect(bearing.frame_b,cylinder. crank_a) annotation (Line(
        points={{-24,60},{-4,60},{-4,74},{2,74}},
        color={95,95,95},
        thickness=0.5));
    connect(bearing.frame_a,world. frame_b) annotation (Line(
        points={{-44,60},{-54,60},{-54,54},{-64,54}},
        color={95,95,95},
        thickness=0.5));
    connect(speedSensor.flange,bearing. axis) annotation (Line(points={{-52,26},
            {-56,26},{-56,44},{-34,44},{-34,50}},                  color={0,0,0}));
    connect(damper.flange_b,fixed. flange)
      annotation (Line(points={{66,62},{78,62},{78,54}},color={0,0,0}));
    connect(dcse.pin_an,dcse. pin_ep) annotation (Line(points={{-30,-46},{-30,
            -36},{-38,-36},{-38,-50},{-34,-50}},
                                            color={0,0,255}));
    connect(dcse.pin_en,ground. p)
      annotation (Line(points={{-34,-62},{-62,-62},{-62,-68}}, color={0,0,255}));
    connect(cellStack.n,ground. p) annotation (Line(points={{-82,-36},{-86,-36},
            {-86,-58},{-62,-58},{-62,-68}},color={0,0,255}));
    connect(bearing.axis,damper. flange_a) annotation (Line(points={{-34,50},{
            -34,44},{34,44},{34,62},{46,62}},
                                          color={0,0,0}));
    connect(inertia.flange_a,damper. flange_a) annotation (Line(points={{10,32},
            {4,32},{4,44},{34,44},{34,62},{46,62}},                      color={0,
            0,0}));
    connect(dcse.flange,oneWayClutch. flange_a)
      annotation (Line(points={{-14,-56},{12,-56},{12,-36},{26,-36}},
                                                   color={0,0,0}));
    connect(const.y,oneWayClutch. f_normalized) annotation (Line(points={{29,-4},
            {38,-4},{38,-18},{36,-18},{36,-25}},  color={0,0,127}));
    connect(booleanPulse.y,switch. control) annotation (Line(points={{-35.1,-1},
            {-24,-1},{-24,-12}},                                         color={
            255,0,255}));
    connect(damper1.flange_a,dcse. flange) annotation (Line(points={{26,-66},{
            -4,-66},{-4,-56},{-14,-56}},
                                      color={0,0,0}));
    connect(fixed1.flange,damper1. flange_b)
      annotation (Line(points={{70,-66},{46,-66}},          color={0,0,0}));
    connect(oneWayClutch.flange_b,idealGear. flange_a) annotation (Line(points={{46,-36},
            {62,-36},{62,-6},{68,-6}},             color={0,0,0}));
    connect(idealGear.flange_b,inertia. flange_b) annotation (Line(points={{88,-6},
            {92,-6},{92,32},{30,32}},       color={0,0,0}));
    connect(fixedTemperature.port,cellStack. heatPort) annotation (Line(points={{-70,-2},
            {-70,-16},{-72,-16},{-72,-26}},           color={191,0,0}));
    connect(cellStack.p,switch. p) annotation (Line(points={{-62,-36},{-40,-36},
            {-40,-24},{-34,-24}}, color={0,0,255}));
    connect(switch.n,dcse. pin_ap) annotation (Line(points={{-14,-24},{-8,-24},
            {-8,-38},{-18,-38},{-18,-46}},  color={0,0,255}));
    annotation ();
  end Winter;
  annotation (uses(Modelica(version="4.0.0")));
end Project1;
