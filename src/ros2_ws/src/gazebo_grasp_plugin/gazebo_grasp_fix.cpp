/*
 * gazebo_grasp_fix.cpp — Physics-honest grasp plugin for Gazebo Classic.
 *
 * Replaces the "weld" (SetEntityState teleport) hack with a real fixed joint
 * created through the physics engine's constraint solver.
 *
 * Algorithm (standard GazeboGraspFix approach):
 *   1. Subscribe to the world contacts topic.
 *   2. Count, per external object, how many finger-collisions are touching it
 *      in a given update tick.
 *   3. If an object has been contacted for >= maxGraspCount consecutive ticks
 *      AND we are not already holding it  -> create a fixed joint between the
 *      configured palm link and the object's link.  The object now rides the
 *      gripper through real constraint forces — no teleporting.
 *   4. Release when the gripper opens past a threshold (read directly from the
 *      gripper_controller joint of our own model) — destroy the joint and let
 *      gravity take over.
 *
 * Configured entirely from SDF tags so it drops into a URDF <gazebo> block:
 *   <gripper_finger_link>  name  (repeatable)   — fingertip links to monitor
 *   <palm_link>            name                  — anchor link for the joint
 *   <gripper_joint>        name  (default: gripper_controller)
 *   <release_position>     rad   (default: 0.0)  — open past this -> release
 *   <grasp_count_threshold> int  (default: 10)   — debounce frames
 */

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math.hh>

#include <algorithm>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace gazebo {

class GazeboGraspFix : public ModelPlugin {
 public:
  GazeboGraspFix() : ModelPlugin() {}

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override {
    this->model_ = _parent;
    this->world_ = this->model_->GetWorld();
    this->physics_ = this->world_->Physics();

    // ---- configurable finger links (repeatable tag) ----
    if (_sdf->HasElement("gripper_finger_link")) {
      sdf::ElementPtr e = _sdf->GetElement("gripper_finger_link");
      while (e) {
        finger_links_.push_back(e->Get<std::string>());
        e = e->GetNextElement("gripper_finger_link");
      }
    }
    if (finger_links_.empty()) {
      gzwarn << "[GazeboGraspFix] No <gripper_finger_link> tags given — "
                "plugin will never detect a grasp.\n";
    }

    // ---- palm anchor link ----
    if (_sdf->HasElement("palm_link"))
      palm_link_name_ = _sdf->Get<std::string>("palm_link");
    else
      palm_link_name_ = "gripper_base";

    // ---- gripper driving joint + release threshold ----
    gripper_joint_name_ = _sdf->HasElement("gripper_joint")
                              ? _sdf->Get<std::string>("gripper_joint")
                              : "gripper_controller";
    release_position_ = _sdf->HasElement("release_position")
                            ? _sdf->Get<double>("release_position")
                            : 0.0;
    grasp_count_threshold_ = _sdf->HasElement("grasp_count_threshold")
                                 ? _sdf->Get<int>("grasp_count_threshold")
                                 : 10;

    // ---- gazebo transport: subscribe to the world contacts topic ----
    node_ = transport::NodePtr(new transport::Node());
    node_->Init();
    // Topic name is world-relative; "~/physics/contacts" is the global one.
    contacts_sub_ = node_->Subscribe("~/physics/contacts",
                                     &GazeboGraspFix::OnContacts, this);

    // ---- per-tick update hook ----
    update_conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboGraspFix::OnUpdate, this));

    gzmsg << "[GazeboGraspFix] loaded on model '" << this->model_->GetName()
          << "'  fingers=" << finger_links_.size()
          << "  palm='" << palm_link_name_
          << "'  gripper_joint='" << gripper_joint_name_
          << "'  release_pos=" << release_position_
          << "  threshold=" << grasp_count_threshold_ << "\n";
  }

 private:
  // ---- contact callback (runs on transport thread) ----
  void OnContacts(const boost::shared_ptr<const msgs::Contacts> &_msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < _msg->contact_size(); ++i) {
      const msgs::Contact &c = _msg->contact(i);
      const std::string &c1 = c.collision1();
      const std::string &c2 = c.collision2();

      const bool f1 = IsFinger(c1);
      const bool f2 = IsFinger(c2);
      if (!f1 && !f2) continue;            // neither side is a finger
      if (f1 && f2) continue;              // two of our own fingers — ignore

      const std::string obj_collision = f1 ? c2 : c1;
      const std::string obj_model = ModelOf(obj_collision);
      if (obj_model.empty() || obj_model == model_->GetName()) continue;

      contact_accum_[obj_model] += 1;       // accumulate within this tick
    }
  }

  // ---- per-tick logic (runs on physics thread) ----
  void OnUpdate() {
    std::lock_guard<std::mutex> lk(mtx_);

    // snapshot & reset this tick's accumulated contacts
    auto this_tick = contact_accum_;
    contact_accum_.clear();

    // ---- update grip / release counters per seen object ----
    std::unordered_set<std::string> seen;
    for (const auto &kv : this_tick) {
      const std::string &obj = kv.first;
      seen.insert(obj);
      if (kv.second > 0) {
        grip_count_[obj] += 1;
        release_count_[obj] = 0;
      }
    }
    // objects not touched this tick slide toward release (only matters pre-grasp)
    for (auto &kv : grip_count_) {
      if (seen.find(kv.first) == seen.end()) {
        kv.second = 0;
        release_count_[kv.first] += 1;
      }
    }

    // ---- if not holding, try to attach ----
    if (!held_obj_link_) {
      // GATE ADDED 2026-08-19: only attach while the gripper is actually CLOSING.
      //
      // Contact alone is not a grasp. The approach descends with the fingers OPEN,
      // and they brush the box on the way in -- enough contacts to pass
      // grasp_count_threshold_ and weld. The release side then immediately sees the
      // very same open gripper (gpos > release_position_) and unwelds, contacts are
      // still present so it re-welds, and the plugin oscillates GRASP ON/OFF
      // indefinitely: hundreds of cycles in a single approach, observed in the
      // 2026-08-19 run log.
      //
      // Each cycle creates and destroys a real fixed joint between the palm and a
      // 0.04 kg box, so each one is a physics discontinuity that kicks the arm. That
      // is the joint-2/joint-3 oscillation and the end-effector moving forward and
      // back while the gripper is still above the box -- it starts BEFORE the pick
      // precisely because open fingers touching is all it needs.
      //
      // The two sides must agree on what "gripping" means. Release fires when
      // gpos > release_position_; attach must therefore be forbidden in that same
      // region, otherwise the two conditions are simultaneously true and fight.
      // The earlier release debounce (grasp_count_threshold_ sustained ticks) only
      // lengthened each cycle -- it treated the symptom, not this contradiction.
      double gpos_attach = 0.0;
      physics::JointPtr gj_attach = model_->GetJoint(gripper_joint_name_);
      if (gj_attach) gpos_attach = gj_attach->Position(0);
      if (gpos_attach > release_position_) {
        // Gripper is open. Whatever the fingers are touching, it is not a grasp.
        // Clear the counters so contacts made while open cannot accumulate and
        // fire the instant the gripper starts to close.
        grip_count_.clear();
        release_count_.clear();
        return;
      }

      // pick the object with the strongest sustained grip
      std::string best_obj;
      int best_count = 0;
      for (const auto &kv : grip_count_) {
        if (kv.second >= grasp_count_threshold_ && kv.second > best_count) {
          // Never weld to a STATIC model. Observed 2026-07-29: the fingers brush the
          // table and the ground on the way in, and the plugin cheerfully welded the
          // palm to 'pickup_table' and 'ground_plane' — which pins the whole arm to
          // the world and is far worse than missing a grasp. Static models are scenery
          // (ground, tables, walls); only free bodies are graspable.
          physics::ModelPtr cand = world_->ModelByName(kv.first);
          if (!cand || cand->IsStatic()) continue;
          best_count = kv.second;
          best_obj = kv.first;
        }
      }
      if (!best_obj.empty()) AttachObject(best_obj);
    } else {
      // ---- holding: release when gripper opens ----
      // Debounced the same way the attach side already is (grasp_count_threshold_
      // sustained ticks). Diagnosed 2026-08-11: a bare single-tick "gpos >
      // release_position_" check has no hysteresis against the held object's own
      // reaction load -- the gripper joint's reported position micro-oscillates
      // around that threshold under load (ordinary servo/joint compliance), so it
      // released, immediately re-contacted (the fingers never actually opened),
      // re-attached, and repeated -- 524 ON/OFF messages in one trial, each a
      // physics discontinuity, which is what was throwing the arm/box around.
      double gpos = 0.0;
      physics::JointPtr gj = model_->GetJoint(gripper_joint_name_);
      if (gj) gpos = gj->Position(0);
      if (gpos > release_position_) {
        release_streak_ += 1;
        if (release_streak_ >= grasp_count_threshold_) ReleaseObject();
      } else {
        release_streak_ = 0;
        // optional sanity: if the object link vanished (deleted), release
        if (!held_obj_link_ || !held_obj_link_->GetParentModel())
          ReleaseObject();
      }
    }
  }

  // ---- create the fixed joint between palm and object link ----
  void AttachObject(const std::string &obj_model_name) {
    physics::ModelPtr obj_model = world_->ModelByName(obj_model_name);
    if (!obj_model) {
      gzerr << "[GazeboGraspFix] object model '" << obj_model_name
            << "' not found in world\n";
      return;
    }
    // grab the object's canonical (first) link
    auto links = obj_model->GetLinks();
    if (links.empty()) {
      gzerr << "[GazeboGraspFix] object '" << obj_model_name
            << "' has no links\n";
      return;
    }
    held_obj_link_ = links[0];

    physics::LinkPtr palm = model_->GetLink(palm_link_name_);
    if (!palm) {
      gzerr << "[GazeboGraspFix] palm link '" << palm_link_name_
            << "' not found on model\n";
      held_obj_link_.reset();
      return;
    }

    // build a fixed joint through the physics engine
    fixed_joint_ = physics_->CreateJoint("fixed", model_);
    fixed_joint_->Load(palm, held_obj_link_, ignition::math::Pose3d());
    fixed_joint_->Init();
    fixed_joint_->SetModel(model_);

    // zero out the object's residual velocity so it doesn't fight the joint
    held_obj_link_->ResetPhysicsStates();
    held_obj_link_->SetLinearVel(ignition::math::Vector3d::Zero);
    held_obj_link_->SetAngularVel(ignition::math::Vector3d::Zero);

    held_obj_name_ = obj_model_name;
    // clear counters so they don't immediately re-trigger
    grip_count_.clear();
    release_count_.clear();
    release_streak_ = 0;

    gzmsg << "[GazeboGraspFix] GRASP ON — fixed joint '" << palm_link_name_
          << "' <-> '" << obj_model_name << "'\n";
  }

  void ReleaseObject() {
    if (fixed_joint_) {
      fixed_joint_->Detach();
      fixed_joint_.reset();
    }
    if (held_obj_link_) {
      // let it drop naturally under gravity
      held_obj_link_.reset();
    }
    if (!held_obj_name_.empty()) {
      gzmsg << "[GazeboGraspFix] GRASP OFF — released '" << held_obj_name_
            << "'\n";
      held_obj_name_.clear();
    }
    grip_count_.clear();
    release_count_.clear();
  }

  // ---- helpers ----
  // collision scoped name looks like  "mbot::gripper_left1::collision"
  bool IsFinger(const std::string &collision_scoped) const {
    for (const auto &fl : finger_links_)
      if (collision_scoped.find(fl) != std::string::npos) return true;
    return false;
  }

  static std::string ModelOf(const std::string &scoped) {
    const auto p = scoped.find("::");
    if (p == std::string::npos) return "";
    return scoped.substr(0, p);
  }

  // ---- members ----
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::PhysicsEnginePtr physics_;

  std::vector<std::string> finger_links_;
  std::string palm_link_name_;
  std::string gripper_joint_name_;
  double release_position_;
  int grasp_count_threshold_;
  int release_streak_ = 0;   // consecutive ticks with gpos > release_position_

  transport::NodePtr node_;
  transport::SubscriberPtr contacts_sub_;
  event::ConnectionPtr update_conn_;

  std::mutex mtx_;
  // object model name -> contact count accumulated in the current tick
  std::unordered_map<std::string, int> contact_accum_;
  std::unordered_map<std::string, int> grip_count_;
  std::unordered_map<std::string, int> release_count_;

  physics::JointPtr fixed_joint_;
  physics::LinkPtr held_obj_link_;
  std::string held_obj_name_;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboGraspFix)

}  // namespace gazebo
