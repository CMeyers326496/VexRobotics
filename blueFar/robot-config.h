vex::brain Brain;
vex::motor drive_RT (vex::PORT12, vex::gearSetting::ratio18_1,true);
vex::motor drive_RTBCK (vex::PORT1, vex::gearSetting::ratio18_1,true);
vex::motor intake_motor (vex::PORT19, vex::gearSetting::ratio18_1,false);
vex::motor drive_LFT (vex::PORT3, vex::gearSetting::ratio18_1,false);
vex::motor drive_LFTBCK (vex::PORT10, vex::gearSetting::ratio18_1,false);
vex::motor shooter_motor (vex::PORT6, vex::gearSetting::ratio18_1,false);
vex::motor shooter_motor2 (vex::PORT7, vex::gearSetting::ratio18_1,false);






vex::digital_in intake_empty (Brain.ThreeWirePort.A);
vex::sonar shooter_empty (Brain.ThreeWirePort.E);
vex::controller vexRT;