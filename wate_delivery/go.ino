void go() {
  for (int i = 268; i < 272; ++i) {
    if (lidar_data[i] != 0) {
      float p = 700 - lidar_data[i];
      float e = e * 0.97 + p;

      float PID = p * 0.4;

      esp.print(String(PID) + " " + String(p) + " " + String(lidar_data[i]) + " " + String(e * 0.001) + " " + String(e));

      if (mode == 1) {
        analogWrite(PWML, 65 - PID);
        analogWrite(PWMR, 65 + PID);
      }
    }

    esp.update();
  }
}