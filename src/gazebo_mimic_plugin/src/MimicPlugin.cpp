// GazeboMimic: a Gazebo Classic ModelPlugin that emulates URDF <mimic> joints.
//
// Gazebo Classic 11 silently drops the <mimic> tag when converting URDF->SDF, so
// mimic joints (e.g. the RoboMaster EP 12-bar gripper finger joints slaved to the
// single actuated gripper_m_joint) never move. This plugin reads mimic relations
// from its OWN <gazebo><plugin> block (which IS preserved) and copies the master
// joint position to each mimic joint every world update:
//
//     angle_mimic = master_position * multiplier + offset
//
// Debug: writes /tmp/mimic_loaded.log on Load and /tmp/mimic_debug.log (periodic)
// with master pos + fingertip distance so calibration is observable without ROS.

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>

#include <string>
#include <vector>
#include <fstream>

namespace gazebo
{

struct MimicDef
{
  physics::JointPtr joint;    // the mimic (slaved) joint
  physics::JointPtr master;   // the actuated master joint
  double multiplier;
  double offset;
};

class GazeboMimic : public ModelPlugin
{
  public:
    GazeboMimic() : cnt(0) {}
    virtual ~GazeboMimic()
    {
      this->update_connection.reset();
    }

    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      this->model = _parent;

      sdf::ElementPtr m = _sdf->GetElement("mimic");
      for (; m != NULL; m = m->GetNextElement("mimic"))
      {
        // URDF/SDF encodes mimic relations as ATTRIBUTES (joint= master= multiplier= offset=),
        // not child elements -- so use HasAttribute/GetAttribute, not HasElement/Get.
        if (!m->HasAttribute("joint") || !m->HasAttribute("master"))
        {
          gzerr << "GazeboMimic: <mimic> requires joint= and master= attributes. Skipping.\n";
          continue;
        }
        std::string jointName, masterName;
        m->GetAttribute("joint")->Get<std::string>(jointName);
        m->GetAttribute("master")->Get<std::string>(masterName);
        double mult = 1.0;
        double off = 0.0;
        if (m->HasAttribute("multiplier"))
        {
          std::string _s; m->GetAttribute("multiplier")->Get<std::string>(_s); mult = std::stod(_s);
        }
        if (m->HasAttribute("offset"))
        {
          std::string _s; m->GetAttribute("offset")->Get<std::string>(_s); off = std::stod(_s);
        }

        physics::JointPtr jj = this->model->GetJoint(jointName);
        physics::JointPtr mj = this->model->GetJoint(masterName);
        if (!jj)
        {
          gzerr << "GazeboMimic: mimic joint " << jointName << " not found in model. Skipping.\n";
          continue;
        }
        if (!mj)
        {
          gzerr << "GazeboMimic: master joint " << masterName << " not found in model. Skipping.\n";
          continue;
        }
        MimicDef d;
        d.joint = jj;
        d.master = mj;
        d.multiplier = mult;
        d.offset = off;
        this->mimics.push_back(d);
        gzmsg << "GazeboMimic: " << jointName << " mimics " << masterName
              << " (angle = master * " << mult << " + " << off << "\n";
      }

      // ---- debug: confirm load ----
      std::ofstream lf("/tmp/mimic_loaded.log", std::ios::trunc);
      if (lf.is_open())
      {
        lf << "LOADED n_mimics=" << this->mimics.size() << "\n";
        for (MimicDef &d : this->mimics)
          lf << "  " << d.joint->GetName() << " x" << d.multiplier << "\n";
        lf.close();
      }

      if (this->mimics.empty())
        gzmsg << "GazeboMimic: no <mimic> entries configured; plugin is idle.\n";

      this->update_connection = event::Events::ConnectWorldUpdateEnd(
          boost::bind(&GazeboMimic::OnUpdate, this));
    }

    void OnUpdate()
    {
      for (MimicDef &d : this->mimics)
      {
        double masterPos = d.master->Position(0);
        double target = masterPos * d.multiplier + d.offset;
        d.joint->SetPosition(0, target);
      }
      if ((++this->cnt % 40) == 0)
      {
        double mp = this->mimics.empty() ? 0.0 : this->mimics[0].master->Position(0);
        physics::LinkPtr lL = this->model->GetLink("left_gripper_link_7");
        physics::LinkPtr lR = this->model->GetLink("right_gripper_link_7");
        double td = -1.0;
        if (lL && lR)
        {
          ignition::math::Vector3d pL = lL->WorldPose().Pos();
          ignition::math::Vector3d pR = lR->WorldPose().Pos();
          td = pL.Distance(pR);
        }
        std::ofstream gf("/tmp/mimic_debug.log", std::ios::trunc);
        if (gf.is_open())
        {
          gf << "master=" << mp << " tip_distance=" << td << "\n";
          for (MimicDef &d : this->mimics)
            gf << "  " << d.joint->GetName() << "=" << d.joint->Position(0) << "\n";
          gf.close();
        }
      }
    }

  private:
    physics::ModelPtr model;
    std::vector<MimicDef> mimics;
    event::ConnectionPtr update_connection;
    int cnt;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboMimic)

}  // namespace gazebo
