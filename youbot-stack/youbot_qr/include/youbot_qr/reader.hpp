#pragma once

#include <cv_bridge/cv_bridge.h>

#include <vector>

namespace qrcode
{

struct QRCode
{
  std::string data;
  double x;
  double y;
  double z;
  QRCode() :
          x(0),
          y(0),
          z(0)
  {
  }
};

/*
 *
 */
class QRCodeReader
{
private:
  std::vector<QRCode> codes;
  double camera_horizontal_fov_deg_; // Degrees
  double sensor_size_;

  void updateSensorSize();

public:
  QRCodeReader() :
          camera_horizontal_fov_deg_(60)
  {
    updateSensorSize();
  }
  virtual ~QRCodeReader();

  /**
   *
   * @param fov Camera horizontal field of view in degrees.
   * @todo Use camera calibration params
   * @return
   */
  QRCodeReader & setFOV(double fov);
  int get(const cv_bridge::CvImageConstPtr & cv_img_ptr);
  std::vector<QRCode> & getQRs();
};

}
