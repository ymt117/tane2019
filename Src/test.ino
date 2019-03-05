// Display IMU sensor value
void imu_test(){
  float pressure = ps.readPressureMillibars();
  mag.read();
  imu.read();

  Serial.print("P: ");
  Serial.print(pressure);
  snprintf(report, sizeof(report),
    " A: %5d %5d G: %5d %5d M: %5d %5d\n",
    imu.a.x, imu.a.y,
    imu.g.x, imu.g.y,
    mag.m.x, mag.m.y);
  Serial.print(report);
}

void gps_test(){
  while(ss.available() > 0)
    gps.encode(ss.read());

  Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
  Serial.print("\t");
  Serial.print("Lng: "); Serial.println(gps.location.lng(), 6);
}