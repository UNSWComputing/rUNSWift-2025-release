#include "motion/generator/ActionGenerator.hpp"
#include "motion/generator/DistributedGenerator.hpp"
#include "motion/generator/GetupGenerator.hpp"
#include "motion/generator/HeadGenerator.hpp"
#include "motion/generator/NullGenerator.hpp"
#include "motion/generator/RefPickupGenerator.hpp"
#include "motion/generator/UnstiffGenerator.hpp"
#include "motion/generator/WalkEnginePreProcessor.hpp"
#include "runswift_interfaces/msg/body_command.hpp"
#include "utils/body.hpp"

using BodyCommand = runswift_interfaces::msg::BodyCommand;
using Body = BodyCommand;

/*-----------------------------------------------------------------------------
 * Distributed Generator
 * ---------------------
 * This generator switches between all required generators as requested.
 *---------------------------------------------------------------------------*/
DistributedGenerator::DistributedGenerator(const rclcpp::Node::SharedPtr & node, bool simulation)
: Generator(node),
  isStopping(false),
  current_generator(Body::NULL_GENERATOR),
  prev_generator(Body::NULL_GENERATOR),
  requestedDive(Body::NULL_GENERATOR)
{

  headGenerator = (Generator *)(new HeadGenerator(node));
  if (!headGenerator) {
    RCLCPP_FATAL(motion_node_->get_logger(), "headGenerator is NULL!");
  }

  // TODO(dpad): Rewrite these ugly llogs to simply loop through bodyGenerators
  // and print out the string name
  bodyGenerators[Body::NULL_GENERATOR] = (Generator *)(new NullGenerator(node));
  if (!bodyGenerators[Body::NULL_GENERATOR]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[NULL_GENERATOR] is NULL!");
  }

  bodyGenerators[Body::STAND] =
    (Generator *)(new ActionGenerator(node, "standing-to/standStraight"));
  if (!bodyGenerators[Body::STAND]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[STAND] is NULL!");
  }

  bodyGenerators[Body::MOTION_CALIBRATE] =
    (Generator *)(new ActionGenerator(node, "sitting-to/standStraight"));

  if (!bodyGenerators[Body::MOTION_CALIBRATE]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[MOTION_CALIBRATE] is NULL!");
  }

  bodyGenerators[Body::WALK] = (Generator *)(new WalkEnginePreProcessor(node));
  if (!bodyGenerators[Body::WALK]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[WALK] is NULL!");
  }

  bodyGenerators[Body::KICK] = bodyGenerators[Body::WALK];

  if (simulation) {
    bodyGenerators[Body::GETUP_FRONT] = new ActionGenerator(node, "fallen-to/FRONT");
  } else {
    bodyGenerators[Body::GETUP_FRONT] = new GetupGenerator(node, "FRONT");
  }
  if (!bodyGenerators[Body::GETUP_FRONT]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[GETUP_FRONT] is NULL!");
  }

  if (simulation) {
    bodyGenerators[Body::GETUP_BACK] = new ActionGenerator(node, "fallen-to/BACK");
  } else {
    bodyGenerators[Body::GETUP_BACK] = new GetupGenerator(node, "BACK");
  }

  if (!bodyGenerators[Body::GETUP_BACK]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[GETUP_BACK] is NULL!");
  }

  bodyGenerators[Body::TIP_OVER] = (Generator *)
    (new ActionGenerator(node, "standing-to/tipOver"));
  if (!bodyGenerators[Body::TIP_OVER]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[TIP_OVER] is NULL!");
  }

  bodyGenerators[Body::INITIAL] = (Generator *)
    (new ActionGenerator(node, "sitting-to/initial"));

  if (!bodyGenerators[Body::INITIAL]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[INITIAL] is NULL!");
  }

  bodyGenerators[Body::UNSTIFF] = (Generator *)(new UnstiffGenerator(node));
  if (!bodyGenerators[Body::UNSTIFF]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[UNSTIFF] is NULL!");
  }

  bodyGenerators[Body::REF_PICKUP] = (Generator *)(new RefPickupGenerator(node));
  if (!bodyGenerators[Body::REF_PICKUP]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[REF_PICKUP] is NULL!");
  }

  bodyGenerators[Body::GOALIE_SIT] = (Generator *)
    (new ActionGenerator(node, "standing-to/goalieSit"));
  if (!bodyGenerators[Body::GOALIE_SIT]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[GOALIE_SIT] is NULL!");
  }

  bodyGenerators[Body::GOALIE_DIVE_LEFT] = (Generator *)
    (new ActionGenerator(node, "standing-to/goalieDiveLeft"));
  if (!bodyGenerators[Body::GOALIE_DIVE_LEFT]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[GOALIE_DIVE_LEFT] is NULL!");
  }

  bodyGenerators[Body::GOALIE_DIVE_RIGHT] = (Generator *)
    (new ActionGenerator(node, "standing-to/goalieDiveRight"));
  if (!bodyGenerators[Body::GOALIE_DIVE_RIGHT]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[GOALIE_DIVE_RIGHT] is NULL!");
  }

  bodyGenerators[Body::GOALIE_CENTRE] = (Generator *)
    (new ActionGenerator(node, "standing-to/goalieCentre"));
  if (!bodyGenerators[Body::GOALIE_CENTRE]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[GOALIE_CENTRE] is NULL!");
  }

  bodyGenerators[Body::GOALIE_UNCENTRE] = (Generator *)
    (new ActionGenerator(node, "dive-centre-to/goalieUncentre"));
  if (!bodyGenerators[Body::GOALIE_UNCENTRE]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[GOALIE_UNCENTRE] is NULL!");
  }

  bodyGenerators[Body::GOALIE_INITIAL] = (Generator *)
    (new ActionGenerator(node, "sitting-to/goalieInitial"));
  if (!bodyGenerators[Body::GOALIE_INITIAL]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[GOALIE_INITIAL] is NULL!");
  }

  bodyGenerators[Body::DEFENDER_CENTRE] = (Generator *)
    (new ActionGenerator(node, "standing-to/defenderCentre"));
  if (!bodyGenerators[Body::DEFENDER_CENTRE]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[DEFENDER_CENTRE] is NULL!");
  }

  bodyGenerators[Body::GOALIE_FAST_SIT] = (Generator *)
    (new ActionGenerator(node, "standing-to/goalieFastSit"));
  if (!bodyGenerators[Body::GOALIE_FAST_SIT]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[GOALIE_FAST_SIT] is NULL!");
  }

  bodyGenerators[Body::RAISE_ARM] = (Generator *)
    (new ActionGenerator(node, "standing-to/raiseArm"));
  if (!bodyGenerators[Body::RAISE_ARM]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[RAISE_ARM] is NULL!");
  }

  bodyGenerators[Body::UKEMI_FRONT] = (Generator *)
    (new ActionGenerator(node, "standing-to/ukemiFront"));
  if (!bodyGenerators[Body::UKEMI_FRONT]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[UKEMI_FRONT] is NULL!");
  }

  bodyGenerators[Body::UKEMI_BACK] = (Generator *)
    (new ActionGenerator(node, "standing-to/ukemiBack"));
  if (!bodyGenerators[Body::UKEMI_BACK]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[UKEMI_BACK] is NULL!");
  }

  bodyGenerators[Body::GOALIE_STAND] = (Generator *)
    (new ActionGenerator(node, "sitting-to/goalieStand"));
  if (!bodyGenerators[Body::GOALIE_STAND]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[GOALIE_STAND] is NULL!");
  }

  bodyGenerators[Body::SIT] = (Generator *)
    (new ActionGenerator(node, "standing-to/sit"));
  if (!bodyGenerators[Body::SIT]) {
    RCLCPP_FATAL(motion_node_->get_logger(), "bodyGenerators[SIT] is NULL!");
  }

  RCLCPP_INFO(
    motion_node_->get_logger(), "DistributedGenerator constructed");
}

/*-----------------------------------------------------------------------------
 * Destructor
 *---------------------------------------------------------------------------*/
DistributedGenerator::~DistributedGenerator()
{
  delete headGenerator;
  for (auto & i : bodyGenerators) {
    if (i.second) {
      delete i.second;
      i.second = nullptr;
    }
  }
  RCLCPP_INFO(
    motion_node_->get_logger(), "DistributedGenerator destroyed");
}

// 3 is the highest priority, one can think of 0 as the default and lowest priority
std::unordered_map<std::string, int> prioritiesMap = {
  {BodyCommand::GETUP_BACK, 3},
  {BodyCommand::GETUP_FRONT, 3},
  {BodyCommand::GOALIE_DIVE_LEFT, 3},
  {BodyCommand::GOALIE_DIVE_RIGHT, 3},
  {BodyCommand::GOALIE_CENTRE, 3},
  {BodyCommand::TIP_OVER, 3},
  {BodyCommand::UKEMI_BACK, 3},
  {BodyCommand::UKEMI_FRONT, 3},
  {BodyCommand::DEFENDER_CENTRE, 2},
  {BodyCommand::GOALIE_FAST_SIT, 2},
  {BodyCommand::GOALIE_SIT, 2},
  {BodyCommand::GOALIE_UNCENTRE, 2},
  {BodyCommand::GOALIE_INITIAL, 2},
  {BodyCommand::INITIAL, 2},
  {BodyCommand::GOALIE_STAND, 1},
  {BodyCommand::RAISE_ARM, 1},
  {BodyCommand::REF_PICKUP, 1},
  {BodyCommand::SIT, 1},
  {BodyCommand::UNSTIFF, 1}
};
/*-----------------------------------------------------------------------------
 * makeJoints
 * Returns the joint values requested by whichever generator we're using
 *---------------------------------------------------------------------------*/
JointValues DistributedGenerator::makeJoints(MakeJointsIn & makeJointsIn)
{
  MotionCommand * request = makeJointsIn.request;

  // If we're requesting a dive, set requestedDive variable
  if (requestedDive == Body::NULL_GENERATOR &&
    !(
      current_generator == Body::GOALIE_CENTRE ||
      current_generator == Body::GOALIE_DIVE_LEFT ||
      current_generator == Body::GOALIE_DIVE_RIGHT ||
      current_generator == Body::DEFENDER_CENTRE
    ) &&
    (
      request->body_command.action_type == BodyCommand::GOALIE_CENTRE ||
      request->body_command.action_type == BodyCommand::GOALIE_DIVE_LEFT ||
      request->body_command.action_type == BodyCommand::GOALIE_DIVE_RIGHT ||
      current_generator == Body::DEFENDER_CENTRE
    ))
  {
    requestedDive = request->body_command.action_type;
  }

  JointValues fromBody;

  // Check the priority of the requested action compared to the current action
  if (prioritiesMap[request->body_command.action_type] > prioritiesMap[current_generator]) {
    reset();
    isStopping = false;
  }

  if (!bodyGenerators[current_generator]->isActive()) {
    if (bodyGenerators[current_generator] != bodyGenerators[request->body_command.action_type] ||
      isStopping ||
      (current_generator == Body::GETUP_FRONT &&
      request->body_command.action_type == BodyCommand::GETUP_FRONT) ||
      (current_generator == Body::GETUP_BACK &&
      request->body_command.action_type == BodyCommand::GETUP_BACK))
    {
      bodyGenerators[current_generator]->reset();
    }

    if (current_generator == Body::GOALIE_CENTRE &&
      request->body_command.action_type != BodyCommand::GOALIE_CENTRE)
    {
      current_generator = Body::GOALIE_UNCENTRE;
    } else if (current_generator == Body::SIT &&
      request->body_command.action_type != BodyCommand::SIT &&
      request->body_command.action_type != BodyCommand::UNSTIFF)
    {
      current_generator = Body::INITIAL; // If we're sitting stand up gracefully
    } else {
      current_generator = request->body_command.action_type;
    }
    isStopping = false;
  } else if (bodyGenerators[current_generator]->isActive() &&
    bodyGenerators[current_generator] !=
    bodyGenerators[request->body_command.action_type])
  {
    // Special case to let kicks continue instead of being interrupted by stand
    if (current_generator != Body::KICK ||
      request->body_command.action_type != BodyCommand::STAND)
    {
      bodyGenerators[current_generator]->stop();
      isStopping = true;
    }
  }

  if (current_generator == requestedDive) {
    requestedDive = Body::NULL_GENERATOR;
  }

  const std::unordered_map<std::string, bool> usesHeadMap = {
    {BodyCommand::GETUP_FRONT, true},
    {BodyCommand::GETUP_BACK, true},
    {BodyCommand::GOALIE_SIT, true},
    {BodyCommand::GOALIE_FAST_SIT, true},
    {BodyCommand::GOALIE_DIVE_LEFT, true},
    {BodyCommand::GOALIE_DIVE_RIGHT, true},
    {BodyCommand::GOALIE_CENTRE, true},
    {BodyCommand::GOALIE_UNCENTRE, true},
    {BodyCommand::GOALIE_INITIAL, true},
    {BodyCommand::INITIAL, true},
    {BodyCommand::RAISE_ARM, true},
    {BodyCommand::SIT, true},
    {BodyCommand::TIP_OVER, true},
    {BodyCommand::UKEMI_BACK, true},
    {BodyCommand::UKEMI_FRONT, true},
    {BodyCommand::UNSTIFF, true},
    {BodyCommand::DEFENDER_CENTRE, false},
    {BodyCommand::GOALIE_STAND, false},
    {BodyCommand::KICK, false},
    {BodyCommand::NULL_GENERATOR, false},
    {BodyCommand::REF_PICKUP, false},
    {BodyCommand::STAND, false},
    {BodyCommand::WALK, false}
  };

  // Robot will not stiffen without this
  fromBody = bodyGenerators[current_generator]->makeJoints(makeJointsIn);

  if (current_generator == Body::KICK && request->body_command.action_type == BodyCommand::WALK) {
    current_generator = Body::WALK;
  }
  if (!usesHeadMap.at(current_generator)) {
    JointValues fromHead = headGenerator->
      makeJoints(makeJointsIn);
    for (uint8_t i = Joints::HeadYaw; i <= Joints::HeadPitch; ++i) {
      fromBody.angles[i] = fromHead.angles[i];
      fromBody.stiffnesses[i] = fromHead.stiffnesses[i];
    }
  }
  prev_generator = current_generator;

  return fromBody;
}

bool DistributedGenerator::isActive()
{
  return true;
}

void DistributedGenerator::reset()
{
  for (auto & i : bodyGenerators) {
    if (i.second) {
      i.second->reset();
    }
  }
  headGenerator->reset();
  current_generator = BodyCommand::NULL_GENERATOR;
}

void DistributedGenerator::readParameters()
{
  for (auto & i : bodyGenerators) {
    if (i.second) {
      i.second->readParameters();
    }
  }
  headGenerator->readParameters();
}
