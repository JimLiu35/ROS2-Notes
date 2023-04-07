# Ignition Gazebo - Create System Plug in

In last node, we talk about how to add the sensor and model into gazebo through urdf file. In this note, I want to introduce to you how to create system plugins. 

[https://gazebosim.org/api/gazebo/4.2/createsystemplugins.html](https://gazebosim.org/api/gazebo/4.2/createsystemplugins.html)

### 1.0 launch the Gazebo

Launch the gazebo through the ros launch, and `-v` stand for verbal

```cpp
ros2 launch ros_ign_gazebo ign_gazebo.launch.py ign_args:="empty.sdf -v 4" 
```

`ign service` in ros2 run

```cpp
ros2 run ros_ign_gazeboo create 
```

### 2.1 system plugin introduction

[Ignition Gazebo: Create System Plugins](https://gazebosim.org/api/gazebo/4.2/createsystemplugins.html)

system plugin version 2.10;

[Ignition Gazebo: Class List](https://gazebosim.org/api/gazebo/2.10/annotated.html)

[Ignition Gazebo: System Class Reference](https://gazebosim.org/api/gazebo/2.10/classignition_1_1gazebo_1_1System.html)

**2.2 Systems are executed in three phases:**

prior virtual function 

- PreUpdate
    - Has read-write access to world entities and components
    - Executed with simulation time at (t0)
    - Can be used to modify state before physics runs, for example for applying control signals or performing network syncronization.
- Update
    - Has read-write access to world entities and components
    - Responsible for propagating time from (t0) to (t0 + dt)
    - Used for physics simulation step
- PostUpdate
    - Has read-only access to world entities and components
    - Executed with simulation time at (t0 + dt)
    - Used to read out results at the end of a simulation step to be used for sensor or controller updates.

Reference link:

| https://gazebosim.org/api/gazebo/2.10/classignition_1_1gazebo_1_1ISystemConfigure.html | Interface for a system that implements optional configuration |
| --- | --- |
| https://gazebosim.org/api/gazebo/2.10/classignition_1_1gazebo_1_1ISystemPostUpdate.html | Interface for a system that uses the PostUpdate phase |
| https://gazebosim.org/api/gazebo/2.10/classignition_1_1gazebo_1_1ISystemPreUpdate.html | Interface for a system that uses the PreUpdate phase |

to implement the above 

<**rsp_plugin>.hpp**

```cpp
#include <ignition/gazebo/System.hh>

class rsp_system:	
	public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate, 
  public ignition::gazebo::ISystemUpdate,
  public ignition::gazebo::ISystemPostUpdate{

public:

  rsp_system();
  ~rsp_system();

  void Configure( const ignition::gazebo::Entity& entity,
		  const std::shared_ptr<const sdf::Element>& sdf, //configuration
		  ignition::gazebo::EntityComponentManager& ecm,
		  ignition::gazebo::EventManager& eventmgr ) override;

  void PreUpdate( const ignition::gazebo::UpdateInfo& info,
		  ignition::gazebo::EntityComponentManager& ecm ) override;
  void Update( const ignition::gazebo::UpdateInfo& info,
	       ignition::gazebo::EntityComponentManager& ecm ) override;
  void PostUpdate( const ignition::gazebo::UpdateInfo& info,
		   const ignition::gazebo::EntityComponentManager& ecm ) override;
  
};
```

**2.2.1 ISystemConfigure Explanation** 

```cpp
Parameters
    [in]	_entity	The entity this plugin is attached to.
    [in]	_sdf	The SDF Element associated with this system plugin.
    [in]	_ecm	The EntityComponentManager of the given simulation instance.
    [in]	_eventMgr	The EventManager of the given simulation instance.
```

**rsp_pulgin.cpp**

```cpp
#include <rsp_week05/rsp_plugin.hpp>
#include <ignition/plugin/Register.hh>

rsp_system::rsp_system(){
  std::cout << "constructor" << std::endl;
}
rsp_system::~rsp_system(){}

void rsp_system::Configure( const ignition::gazebo::Entity& entity,
			    const std::shared_ptr<const sdf::Element>& sdf,
			    ignition::gazebo::EntityComponentManager& ecm,
			    ignition::gazebo::EventManager& eventmgr ){
  std::cout << "configure" << std::endl;

}

void rsp_system::PreUpdate( const ignition::gazebo::UpdateInfo& info,
			    ignition::gazebo::EntityComponentManager& ecm ){
  std::cout << "preupdate" << std::endl;
  
  }
}
void rsp_system::Update( const ignition::gazebo::UpdateInfo& info,
			 ignition::gazebo::EntityComponentManager& ecm ){
  std::cout << "update " << ecm.EntityCount() << std::endl; // give the number of entities https://gazebosim.org/api/gazebo/2.10/classignition_1_1gazebo_1_1EntityComponentManager.html#a4a34d566628f47ade9145d22a5c4947e 

  
}

void rsp_system::PostUpdate( const ignition::gazebo::UpdateInfo& info,
       const ignition::gazebo::EntityComponentManager& ecm ){
  std::cout << "postupdate" << std::endl;
 }

//help it identify itself
IGNITION_ADD_PLUGIN( rsp_system,
		     ignition::gazebo::System,
		     rsp_system::ISystemConfigure,
		     rsp_system::ISystemPreUpdate,
		     rsp_system::ISystemUpdate,
		     rsp_system::ISystemPostUpdate )
```

<**CMakeList>.txt**

```cpp
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ignition-cmake2 REQUIRED)

include_directories( include )

ign_find_package( ignition-plugin1 REQUIRED COMPONENTS register )
ign_find_package( ignition-gazebo6 REQUIRED )

set( IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR} )
set( IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR} )

add_library( rsp_plugin SHARED src/rsp_plugin.cpp )
set_property(TARGET rsp_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries( rsp_plugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER} )

install( DIRECTORY urdf DESTINATION share/${PROJECT_NAME} )
install( TARGETS rsp_plugin LIBRARY DESTINATION lib )
```

**2.3 Check the library**

```cpp
ls install/rsp_week05/lib 
> librsp_plugin.so
```

### 3.0 Control the joint state in sin(t)

[Ignition Gazebo: physics Directory Reference](https://gazebosim.org/api/gazebo/6.7/dir_0c4127926c10b8932bf02ceb115c323c.html)

3.1 add the plugin in `robot.xacro`

```cpp
<gazebo>
      <plugin filename="rsp_plugin" name="rsp_system">
				<max_velocity> 1 </max_velocity>
				<joint> ${prefix}_shoulder </joint> // link with shoulder joint 
				<topic> enable </topic>
      </plugin>
</gazebo>
```

3.2 
Updated **`rsp_pulgin.hpp`**

```cpp
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh> // include for the node 
#include <ignition/gazebo/Model.hh>

class rsp_system:
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate,
  public ignition::gazebo::ISystemUpdate,
  public ignition::gazebo::ISystemPostUpdate {

private:

  ignition::transport::Node node; // create an node, subscriber 

// gazebot link is called as entity # 
  ignition::gazebo::Entity joint; // create an entity: joint
  std::string joint_name; // joint name 

  double joint_speed{0}; // initialize the joint speed as zero
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity}; // create an empty model (Indicates a non-existant or invalid Entity.)

  bool enabled; //flag to activate the joint (turn on/off the robot)

public:

  rsp_system(); //constructor 
  ~rsp_system();

  void callback( const ignition::msgs::Boolean& msg );
//send true/false msg to activate the robot 
  
  void Configure( const ignition::gazebo::Entity& entity,
		  const std::shared_ptr<const sdf::Element>& sdf,
		  ignition::gazebo::EntityComponentManager& ecm,
		  ignition::gazebo::EventManager& eventmgr ) override;

  void PreUpdate( const ignition::gazebo::UpdateInfo& info,
		  ignition::gazebo::EntityComponentManager& ecm ) override;
  void Update( const ignition::gazebo::UpdateInfo& info,
	       ignition::gazebo::EntityComponentManager& ecm ) override;
  void PostUpdate( const ignition::gazebo::UpdateInfo& info,
		   const ignition::gazebo::EntityComponentManager& ecm ) override;
  
};
```

update **`rsp_pulgin.cpp`**

```cpp
#include <rsp_week05/rsp_plugin.hpp>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/JointVelocityCmd.hh>

rsp_system::rsp_system(){
  std::cout << "constructor" << std::endl;
  enabled=true;
  joint = 0;
}
rsp_system::~rsp_system(){}

void rsp_system::Configure( const ignition::gazebo::Entity& entity,
			    const std::shared_ptr<const sdf::Element>& sdf,
			    ignition::gazebo::EntityComponentManager&,
			    ignition::gazebo::EventManager&){
  std::cout << "configure" << std::endl;
  model = ignition::gazebo::Model(entity);//model the entity 

  std::cout << sdf->ToString("element") << std::endl; 
// print out the sdf element 
//[ign gazebo-1] element<plugin name='rsp_system' filename='rsp_plugin'>
//[ign gazebo-1] element  <max_velocity>1</max_velocity>
//[ign gazebo-1] element  <joint>left_shoulder</joint>
//[ign gazebo-1] element  <topic>enable</topic>
//[ign gazebo-1] element</plugin>

  if( sdf->HasElement("max_velocity") ){
    std::cout << "max velocity" << std::endl;
    const double maxVel = sdf->Get<double>("max_velocity");
  }

  if( sdf->HasElement("topic") ){
    std::string topic = sdf->Get<std::string>("topic");
    node.Subscribe(topic, &rsp_system::callback, this ); //subscribe the node
  }

  auto ptr = const_cast<sdf::Element*>(sdf.get());
  if( sdf->HasElement("joint") ){
    joint_name = ptr->GetElement("joint")->Get<std::string>();
    std::cout << joint_name << std::endl; // "left_shoulder"
  }
  
}

void rsp_system::callback( const ignition::msgs::Boolean& msg ){
  std::cout << msg.data() << std::endl; // print out 
  enabled = msg.data();
}

void rsp_system::PreUpdate( const ignition::gazebo::UpdateInfo&,
			    ignition::gazebo::EntityComponentManager& ecm ){
  if( joint == 0 ){
    joint = model.JointByName( ecm, joint_name );
    std::cout << "Model: " << model.Name(ecm)
	      << " has " << model.LinkCount(ecm)
	      << " links and " << model.JointCount(ecm)
	      << " joints " << std::endl;
  } // get the information about the model.

  auto vel = ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint);
  if( vel == nullptr ){
    ecm.CreateComponent(joint,ignition::gazebo::components::JointVelocityCmd({joint_speed}));
  }
  else{
    *vel = ignition::gazebo::components::JointVelocityCmd({joint_speed}); //pointer of the velocity
  }
  
}

void rsp_system::Update( const ignition::gazebo::UpdateInfo&,
			 ignition::gazebo::EntityComponentManager& ){}

void rsp_system::PostUpdate( const ignition::gazebo::UpdateInfo&,
			     const ignition::gazebo::EntityComponentManager&){
  static double t=0.0;
  if( enabled ){
    joint_speed = sin(t);
    t+=0.001;
  }
  else{
    joint_speed = 0.0;
  }
  
}

IGNITION_ADD_PLUGIN( rsp_system,
		     ignition::gazebo::System,
		     rsp_system::ISystemConfigure,
		     rsp_system::ISystemPreUpdate,
		     rsp_system::ISystemUpdate,
		     rsp_system::ISystemPostUpdate )
```

**3.3 Command to run the plug in** 

```cpp
// open the gazebo 
ros2 launch ros_ign_gazebo ign_gazebo.launch.py ign_args:="empty.sdf -v 4" v
```

```cpp
//load the robot
ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename:"/home/huan/test_ws/src/lecture/rsp_week05/urdf/robot.urdf", name:"RSP"'
data: true
```

```cpp
//run the bridge 
ros2 run ros_ign_bridge parameter_bridge /enable@std_msgs/msg/Bool]ignition.msgs.Boolean
```

```cpp
//publish the topic 
ros2 topic pub -1 /enable std_msgs/msg/Bool data:\ true
```