# Galbot S1 USD 检查报告

- 文件: `/root/gpufree-data/galbot_ws/third_party/galbot_s1_description/usd/galbot_s1.usda`
- metersPerUnit: 1.0
- upAxis: Z
- defaultPrim: /galbot_s1_v2_1_0_A21_ge103in

## 变体 (variantSets)

- Robot = robot
- Sensor = none
- Physics = physx

## Articulation Root

- /galbot_s1_v2_1_0_A21_ge103in/root_joint

## 关节 (名称 | 类型 | 轴 | 下限 | 上限 | Drive类型 | stiffness | damping | maxForce | maxVelocity)

- `torso_lift_joint1` | PrismaticJoint | lo=0 hi=0.735 | 
- `head_joint1` | RevoluteJoint | lo=-45 hi=45 | 
- `head_joint2` | RevoluteJoint | lo=-35 hi=35 | 
- `left_arm_joint1` | RevoluteJoint | lo=-168.5 hi=168.5 | 
- `left_arm_joint2` | RevoluteJoint | lo=-113.5 hi=98.5 | 
- `left_arm_joint3` | RevoluteJoint | lo=-167.5 hi=167.5 | 
- `left_arm_joint4` | RevoluteJoint | lo=-148.5 hi=108.5 | 
- `left_arm_joint5` | RevoluteJoint | lo=-167.5 hi=167.5 | 
- `left_arm_joint6` | RevoluteJoint | lo=-43.5 hi=43.5 | 
- `left_arm_joint7` | RevoluteJoint | lo=-89.5 hi=89.5 | 
- `left_active_joint1` | RevoluteJoint | lo=0 hi=93.72 | 
- `left_active_joint2` | RevoluteJoint | lo=-112.5 hi=18.74 | 
- `left_passive_joint` | RevoluteJoint | lo=-18.74 hi=112.5 | 
- `right_arm_joint1` | RevoluteJoint | lo=-168.5 hi=168.5 | 
- `right_arm_joint2` | RevoluteJoint | lo=-98.5 hi=113.5 | 
- `right_arm_joint3` | RevoluteJoint | lo=-167.5 hi=167.5 | 
- `right_arm_joint4` | RevoluteJoint | lo=-108.5 hi=148.5 | 
- `right_arm_joint5` | RevoluteJoint | lo=-167.5 hi=167.5 | 
- `right_arm_joint6` | RevoluteJoint | lo=-43.5 hi=43.5 | 
- `right_arm_joint7` | RevoluteJoint | lo=-89.5 hi=89.5 | 
- `right_active_joint1` | RevoluteJoint | lo=0 hi=93.72 | 
- `right_active_joint2` | RevoluteJoint | lo=-112.5 hi=18.74 | 
- `right_passive_joint` | RevoluteJoint | lo=-18.74 hi=112.5 | 
- `wheel_steering_joint1` | RevoluteJoint | lo=-166.9 hi=166.9 | 
- `wheel_driving_joint1` | RevoluteJoint | lo=-inf hi=inf | 
- `wheel_steering_joint2` | RevoluteJoint | lo=-166.9 hi=166.9 | 
- `wheel_driving_joint2` | RevoluteJoint | lo=-inf hi=inf | 
- `wheel_steering_joint3` | RevoluteJoint | lo=-166.9 hi=166.9 | 
- `wheel_driving_joint3` | RevoluteJoint | lo=-inf hi=inf | 
- `wheel_steering_joint4` | RevoluteJoint | lo=-166.9 hi=166.9 | 
- `wheel_driving_joint4` | RevoluteJoint | lo=-inf hi=inf | 
- `root_joint` | FixedJoint | lo=- hi=- | 

可动关节总数: 32

## 物理

- CollisionAPI prims: 30
- MassAPI prims: 32
- PhysicsScene: ['/PhysicsScene']
- 几何 prims (Mesh/Cube/...): 64

## 相机挂点

- left_arm_camera_link: /galbot_s1_v2_1_0_A21_ge103in/left_arm_link7/left_arm_camera_link
- right_arm_camera_link: /galbot_s1_v2_1_0_A21_ge103in/right_arm_link7/right_arm_camera_link

## 轮关节核对

- wheel_steering_joint1: OK
- wheel_driving_joint1: OK
- wheel_steering_joint2: OK
- wheel_driving_joint2: OK
- wheel_steering_joint3: OK
- wheel_driving_joint3: OK
- wheel_steering_joint4: OK
- wheel_driving_joint4: OK
