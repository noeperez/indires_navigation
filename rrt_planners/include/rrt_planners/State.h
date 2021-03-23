#ifndef RRT_STATE_
#define RRT_STATE_


namespace RRT
{
/**
*	\brief Class representing a state in 6DoF
*
*/
class State
{
public:
  State();
  State(float x, float y, float z = 0.0, float yaw = 0.0, float roll = 0.0,
        float pitch = 0.0, float lv = 0.0, float av = 0.0);
  ~State();

  void getState(float &x, float &y, float &z, float &yaw, float &roll, float &pitch,
                float &lv, float &av);
  float getX() const;
  float getY() const;
  float getZ() const;
  float getYaw() const;
  float getRoll() const;
  float getPitch() const;
  float getLinVel() const;
  float getAngVel() const;

  void setX(float x);
  void setY(float y);
  void setZ(float z);
  void setYaw(float yaw);
  void setRoll(float roll);
  void setPitch(float pitch);
  void setLv(float lv);
  void setAv(float av);

  float operator()(int index) const
  {
    switch (index)
    {
      case 0:
        return x_;
      case 1:
        return y_;
      case 2:
        return z_;
      case 3:
        return yaw_;
      case 4:
        return roll_;
      case 5:
        return pitch_;
      case 6:
        return lin_vel_;
      case 7:
        return ang_vel_;
      default:
        return x_;
    }
  }
  float operator[](int index) const
  {
    switch (index)
    {
      case 0:
        return x_;
      case 1:
        return y_;
      case 2:
        return z_;
      case 3:
        return yaw_;
      case 4:
        return roll_;
      case 5:
        return pitch_;
      case 6:
        return lin_vel_;
      case 7:
        return ang_vel_;
      default:
        return x_;
    }
  }
  bool operator==(const State &other) const
  {
    return (x_ == other.x_ && y_ == other.y_ && z_ == other.z_);
  }

private:
  float x_;

  float y_;

  float z_;

  float yaw_;

  float roll_;

  float pitch_;

  float lin_vel_;

  float ang_vel_;
};
}
#endif
