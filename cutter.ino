void setupCutter()  {
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(RPWM, OUTPUT);
  DPRINT(F("Initial Cutter Motor Pins"));
}

// Turns the Cutter Motor ON
void Motor_Action_Spin_Blades()  {
  if (Cutting_Blades_Activate == 1) {                                       // Blades are turn ON in settings and will spin!
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
    delay(20);
    analogWrite(RPWM, PWM_Blade_Speed);
    DPRINT(F("Cutter:ON_|"));
  }

  if (Cutting_Blades_Activate == 0) {                                     // Blades are turn off in settings and will not spin!
    void stopCutter();
  }
}


// Turns the Cutter Motor OFF
void stopCutter()  {
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  DPRINT(F("Cutter:0FF"));
}
