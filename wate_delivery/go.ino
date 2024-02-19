void go() {
  for (int i = 268; i < 272; ++i) {
    if (lidar_data[i] != 0) {
      float p = 900 - lidar_data[i];
      // float e = e * 0.97 + p;
      float d = p - eold;

      float PID = p * 0.08 + d * 0.03;

      PID = constrain(PID, -10, 10);
      
      eold = p;

      esp.print(String(PID) + " " + String(p) + " " + String(lidar_data[i]) + " " + String(d * 0.03) + " " + String(d));

      if (mode == 1) {
        analogWrite(PWML, 65 - PID);
        analogWrite(PWMR, 65 + PID);
      }
      esp.update();
    }

    esp.update();
  }
}