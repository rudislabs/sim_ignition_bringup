from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, DeclareLaunchArgument, SetLaunchConfiguration, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node

import os
from time import sleep
import json
import numpy as np
import shlex
import sys
import subprocess

# Set relative paths to real paths
launchPath = os.path.realpath(__file__).replace("sim_ignition.launch.py","")
jsonPath = os.path.realpath(os.path.relpath(os.path.join(launchPath,"../config")))
workSpaceROS2 = os.path.realpath(os.path.relpath(os.path.join(launchPath,"../../..")))

#Default to safe
worldFilePath="empty.sdf"

# Initialize flags
setTmpIgnitionResourceEnv=False
hasBuiltROS2Pkgs=False
cleanStart=True
debugVerbose=False
overideROS2Ignition=False

# Clear /tmp output from previous runs
if cleanStart:
    print("INFO: Cleaning previous temporary models")
    os.system("rm -rf /tmp/fuel")

# Open JSON configuration file
with open('{:s}/gen_params.json'.format(jsonPath)) as jsonFile:
    jsonParams = json.load(jsonFile)

#Assume nothing is in json
setupFuel=setupSystem=setupROS2=world=models=ROS2Nodes = None

# Setup related configs
if "setup" in jsonParams:
    setup = jsonParams["setup"]
    if "fuel" in setup:
        setupFuel = setup["fuel"]
    if "system" in setup:
        setupSystem = setup["system"]
    if "ros2" in setup:
        setupROS2 = setup["ros2"]


# Runtime related params
if "nodes" in jsonParams:
    ROS2Nodes= jsonParams["nodes"]
if "world" in jsonParams:
    world = jsonParams["world"]
if "models" in jsonParams:
    models = jsonParams["models"]

########################################################################################

if setupFuel is not None:
    # Iterate through defined fuel model repos
    for repo in setupFuel["fuelModels"]:
        fuelRepo = setupFuel["fuelModels"][repo]
        fuelRepoPath = '{:s}/{:s}'.format(workSpaceROS2, 
            fuelRepo["name"])

        # Clone fuel model repo if not present
        if not os.path.isdir(fuelRepoPath):
            cmdClone = 'git clone -b {:s} {:s} {:s}'.format(
                fuelRepo["version"],fuelRepo["repo"], fuelRepoPath)
            cmdClonePopen=shlex.split(cmdClone)
            clonePopen = subprocess.Popen(cmdClonePopen, 
                stdout=subprocess.PIPE, text=True)
            while True:
                output = clonePopen.stdout.readline()
                if output == '' and clonePopen.poll() is not None:
                    break
                if output:
                    print(output.strip())
            clonePopen.wait()

        # Handle complicated model paths
        if fuelRepoPath.endswith("/models"):
            fuelRepoPath = str(os.path.realpath(os.path.join(fuelRepoPath,'..')))
            if os.path.isdir(os.path.join(fuelRepoPath,'models/.git')):
                os.system('rm -rf {:s}/models/.git'.format(fuelRepoPath))

        # Handle pulling large world models
        if fuelRepoPath.endswith("_worlds"):
            if not os.path.isfile(str(os.path.realpath(os.path.join(fuelRepoPath,
                'models/.dlm')))) and os.path.isfile(str(os.path.realpath(os.path.join(fuelRepoPath,
                'scripts/pull_media.py')))):
                print('''\nUpdating media files in: {:s},
        this might take a while, please be patient.
        This can also be run manually with:
        "python3 {:s}"\n'''.format(fuelRepo["name"],
                        str(os.path.realpath(os.path.join(fuelRepoPath,
                            'scripts/pull_media.py')))))
                cmdDLM = 'python3 {:s}'.format(str(os.path.realpath(os.path.join(fuelRepoPath,'scripts/pull_media.py'))))
                cmdDLMPopen=shlex.split(cmdDLM)
                dlmPopen = subprocess.Popen(cmdDLMPopen, 
                    stdout=subprocess.PIPE, text=True)
                while True:
                    output = dlmPopen.stdout.readline()
                    if output == '' and dlmPopen.poll() is not None:
                        break
                    if output:
                        print(output.strip())
                dlmPopen.wait()


        # Check to see if fuel model path environment variable is reset yet, if not reset to avoid other models
        if not setTmpIgnitionResourceEnv:
            if os.getenv('IGN_GAZEBO_RESOURCE_PATH'):
                os.environ['IGN_GAZEBO_RESOURCE_PATH'] = '/tmp/fuel/models:/tmp/fuel/worlds:{:s}'.format(
                    os.getenv('IGN_GAZEBO_RESOURCE_PATH'))
            else:
                os.environ['IGN_GAZEBO_RESOURCE_PATH'] = '/tmp/fuel/models:/tmp/fuel/worlds'
            setTmpIgnitionResourceEnv=True

        # Append to fuel model/world path environment for subsequent repos if not present
        if os.path.isdir(os.path.join(fuelRepoPath,'models')) and setTmpIgnitionResourceEnv and ('{:s}/models'.format(fuelRepoPath) not in os.getenv('IGN_GAZEBO_RESOURCE_PATH')):
            os.environ['IGN_GAZEBO_RESOURCE_PATH'] = '{:s}/models:{:s}'.format(
                fuelRepoPath, os.getenv('IGN_GAZEBO_RESOURCE_PATH'))
        if os.path.isdir(os.path.join(fuelRepoPath,'worlds')) and setTmpIgnitionResourceEnv and ('{:s}/worlds'.format(fuelRepoPath) not in os.getenv('IGN_GAZEBO_RESOURCE_PATH')):
            os.environ['IGN_GAZEBO_RESOURCE_PATH'] = '{:s}/worlds:{:s}'.format(
                fuelRepoPath, os.getenv('IGN_GAZEBO_RESOURCE_PATH'))
        if not os.path.isdir(os.path.join(fuelRepoPath,'worlds')) and not os.path.isdir(os.path.join(fuelRepoPath,'models')) and setTmpIgnitionResourceEnv and ('{:s}/worlds'.format(fuelRepoPath) not in os.getenv('IGN_GAZEBO_RESOURCE_PATH')):
            os.environ['IGN_GAZEBO_RESOURCE_PATH'] = '{:s}:{:s}'.format(
                fuelRepoPath, os.getenv('IGN_GAZEBO_RESOURCE_PATH'))

########################################################################################
if setupSystem is not None:
    # Set environment variables if present
    if "setEnvironment" in setupSystem:
        for env in setupSystem["setEnvironment"]:
            setEnv = setupSystem["setEnvironment"][env]
            if setEnv["method"] == "overwrite":
                os.environ[setEnv["variable"]] = setEnv["value"]
            elif setEnv["method"] == "prepend":
                if os.getenv(setEnv["variable"]) is None:
                    os.environ[setEnv["variable"]] = setEnv["value"]
                else:
                    os.environ[setEnv["variable"]] = '{:s}:{:s}'.format(
                        setEnv["value"], os.getenv(setEnv["variable"]))
            elif setEnv["method"] == "postpend":
                if os.getenv(setEnv["variable"]) is None:
                    os.environ[setEnv["variable"]] = setEnv["value"]
                else:
                    os.environ[setEnv["variable"]] = '{:s}:{:s}'.format(
                        os.getenv(setEnv["variable"]), setEnv["value"])

    # Always use default if none set
    if os.getenv('IGN_GAZEBO_RESOURCE_PATH') is None:
        print('\nWarning: IGN_GAZEBO_RESOURCE_PATH is not set\n')

    if debugVerbose:
        print('IGN_GAZEBO_RESOURCE_PATH= {:s}'.format(os.getenv('IGN_GAZEBO_RESOURCE_PATH')))

########################################################################################

if setupROS2 is not None:
    for build in setupROS2:
        buildROS2Pkg = setupROS2[build]
        pathROS2Pkg = '{:s}/src/{:s}'.format(workSpaceROS2,buildROS2Pkg["build"]["package"])
        if (not os.path.isdir(pathROS2Pkg)):
            hasBuiltROS2Pkgs=True
            cmdClone = 'git clone -b {:s} {:s} {:s}'.format(
                buildROS2Pkg["version"],buildROS2Pkg["repo"], pathROS2Pkg)
            cmdClonePopen=shlex.split(cmdClone)
            clonePopen = subprocess.Popen(cmdClonePopen, 
                stdout=subprocess.PIPE, text=True)
            while True:
                output = clonePopen.stdout.readline()
                if output == '' and clonePopen.poll() is not None:
                    break
                if output:
                    print(output.strip())
            clonePopen.wait()

            build_cmd = 'colcon build {:s} {:s} {:s}'.format(
                buildROS2Pkg["build"]["prefix"],buildROS2Pkg["build"]["package"],
                buildROS2Pkg["build"]["postfix"])
            build_cmdPopen=shlex.split(build_cmd)
            buildPopen = subprocess.Popen(build_cmdPopen, stdout=subprocess.PIPE, 
                cwd=workSpaceROS2, text=True)
            while True:
                output = buildPopen.stdout.readline()
                if output == '' and buildPopen.poll() is not None:
                    break
                if output:
                    print(output.strip())
            buildPopen.wait()

    # Require restart if colcon packages built so they can be correctly found
    if hasBuiltROS2Pkgs:
        os.system('/bin/bash {:s}/install/setup.bash'.format(workSpaceROS2))
        os.system("/bin/bash /opt/ros/galactic/setup.bash")
        print('''\n\n\nPLEASE RUN:\n 
            source {:s}/install/setup.bash; source /opt/ros/galactic/setup.bash; ros2 launch sim_ignition_bringup sim_ignition.launch.py
            \n\n'''.format(workSpaceROS2))
        sys.exit()

########################################################################################

# Generate the world
if world is not None:
    if 'name' not in world:
        world["name"]="NotSet"

    if 'params' in world:
        worldArgs=' --name "{:s}"'.format(world["name"])
        for param in world["params"]:
            worldArgs += ' --{:s} "{:s}"'.format(param, 
                str(world["params"][param]))

        cmdWorldGen = 'python3 {:s}/{:s}/scripts/jinja_world_gen.py{:s}'.format(
            workSpaceROS2, world["fuelName"], worldArgs
            ).replace("\n","").replace("    ","")
        if debugVerbose:
            print('\nGenerating world with: {:s}\n'.format(cmdWorldGen))
        cmdWorldPopen=shlex.split(cmdWorldGen)
        worldPopen = subprocess.Popen(cmdWorldPopen, stdout=subprocess.PIPE, text=True)
        while True:
            output = worldPopen.stdout.readline()
            if output == '' and worldPopen.poll() is not None:
                break
            if output:
                print(output.strip())
        worldPopen.wait()

        worldFilePath='/tmp/fuel/worlds/{:s}.sdf'.format(world["name"])

        worldLatitude = world["params"]["WGS84"]["degLatitude"]
        worldLongitude = world["params"]["WGS84"]["degLongitude"]
        worldAltitude = world["params"]["WGS84"]["mAltitude"]
        if debugVerbose:
            print('World latitude: {:s}, longitude: {:s} altitude: {:s}'.format(
                str(worldLatitude), str(worldLongitude), str(worldAltitude)))

    else:
        worldFilePath='{:s}/{:s}/worlds/{:s}.sdf'.format(workSpaceROS2,
            world["fuelName"],world["name"])

    # Set relevant world values


    if overideROS2Ignition:
        cmdIgnition = 'ign gazebo {:s} -v 4'.format(str(worldFilePath))
        cmdIgnitionPopen=shlex.split(cmdIgnition)
        ignitionPopen = subprocess.Popen(cmdIgnitionPopen, 
            stdout=subprocess.PIPE, text=True)


########################################################################################

if models is not None:
    # Generate model files
    for model in models:

        modelParams = models[model]["params"]
        instance = int(models[model]["instance"])

        # Assign unique model name if not explicitly set in JSON
        if modelParams["name"] == "NotSet":
            modelParams["name"] = '{:s}_{:d}'.format(
                modelParams["baseModel"], instance)


        # Reset model generation args and pull new ones from JSON
        modelArgs = ""
        for params in modelParams:
            modelArgs += ' --{:s} "{:s}"'.format(
                params, str(modelParams[params]))

        # Model generation command using scripts/jinja_model_gen.py
        cmdModel = 'python3 {:s}/{:s}/scripts/jinja_model_gen.py{:s}'.format(
            workSpaceROS2, models[model]["fuelName"], 
            modelArgs).replace("\n","").replace("    ","")

        if debugVerbose:
            print('\nGenerating model with: {:s}\n'.format(cmdModel))
        cmdModelPopen=shlex.split(cmdModel)
        modelPopen = subprocess.Popen(cmdModelPopen, stdout=subprocess.PIPE, text=True)
        while True:
            output = modelPopen.stdout.readline()
            if output == '' and modelPopen.poll() is not None:
                break
            if output:
                print(output.strip())
        modelPopen.wait()

########################################################################################

def generate_launch_description():
    launchConfigWorld=LaunchConfiguration('world')
    ld = LaunchDescription([
        # World path argument
        DeclareLaunchArgument(
            'world', default_value=worldFilePath,
            description='Full world.sdf name'),
        ])

    if not overideROS2Ignition:
        # Get path to ignition package
        pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

        # Launch ignition with world file from world arg
        ignGazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
                launch_arguments={
                    'ign_args': '-r {:s} -v 4'.format(worldFilePath)
        }.items(),
        )

        ld.add_action(ignGazebo)

    ####################################################################################
    if ROS2Nodes is not None:
        # pre-spawn ROS2 Nodes
        for ROS2Node in ROS2Nodes:
            node = ROS2Nodes[ROS2Node]

            # Run if timing set to pre-spawn of model
            if node["timing"] == "pre-spawn":
                if "remappings" in node:
                    remappings = [eval(str(node["remappings"]))]
                    preSpawnNode = Node(package=node["package"],
                        executable=node["executable"],
                        name=str(node["name"]), 
                        output=node["output"],
                        parameters=node["parameters"],
                        remappings=remappings)
                else:
                    preSpawnNode = Node(package=node["package"],
                        executable=node["executable"],
                        name=str(node["name"]), 
                        output=node["output"],
                        parameters=node["parameters"])

                ld.add_action(preSpawnNode)

    ####################################################################################

    if models is not None:
        # Initialize models
        for model in models:

            modelParams = models[model]["params"]
            instance = int(models[model]["instance"])

            # Assign unique model name if not explicitly set in JSON
            if modelParams["name"] == "NotSet":
                modelParams["name"] = '{:s}_{:d}'.format(
                    modelParams["baseModel"], instance)

            # Calculate spawn locations
            pose = models[model]["pose"]
            modelLatitude = float(worldLatitude) + ((float(pose[1])/6378137.0)*(180.0/np.pi))
            modelLongitude = float(worldLongitude) + ((float(pose[0])/
                (6378137.0*np.cos((np.pi*float(worldLatitude))/180.0)))*(180.0/np.pi))
            modelAltitude = float(worldAltitude) + float(pose[2])
           

            if not overideROS2Ignition:
                # Spawn model in ignition
                spawnEntity = Node(package='ros_ign_gazebo', executable='create',
                                arguments=['-name', '{:s}'.format(modelParams["name"]),
                                    '-world', '{:s}'.format(modelParams["name"]),
                                    '-X', str(pose[0]), '-Y', str(pose[1]), '-Z', str(pose[2]),
                                    '-Roll', str(pose[3]), '-Pitch', str(pose[4]), '-Yaw', str(pose[5]),
                                    '-file', '/tmp/fuel/models/{:s}/model.sdf'.format(
                                        modelParams["name"])],
                                name='spawn_{:s}'.format(modelParams["name"]), output='screen')

                ld.add_action(spawnEntity)

            # if overideROS2Ignition:
            #     cmdSpawn = '''gz model --spawn-file=/tmp/fuel/models/{:s}/model.sdf --model-name={:s} 
            #         -x {:s} -y {:s} -z {:s} -R {:s} -P {:s} -Y {:s}'''.format(
            #         modelParams["name"], modelParams["name"],
            #         modelParams["name"], str(pose[0]),str(pose[1]),
            #         str(pose[2]), str(pose[3]), str(pose[4]), str(pose[5])
            #         ).replace("\n","").replace("    ","")
            #     cmdSpawnPopen=shlex.split(cmdSpawn)
            #     spawnPopen = subprocess.Popen(cmdSpawnPopen, 
            #         stdout=subprocess.PIPE, text=True)

    ####################################################################################

    if ROS2Nodes is not None:
        # post-spawn ROS2 Nodes
        for ROS2Node in ROS2Nodes:
            node = ROS2Nodes[ROS2Node]

            # Run if timing set to post-spawn of model
            if node["timing"] == "post-spawn":
                if "remappings" in node:
                    remappings = [eval(str(node["remappings"]))]
                    postSpawnNode = Node(package=node["package"],
                        executable=node["executable"],
                        name=str(node["name"]), 
                        output=node["output"],
                        parameters=node["parameters"],
                        remappings=remappings)
                else:
                    postSpawnNode = Node(package=node["package"],
                        executable=node["executable"],
                        name=str(node["name"]), 
                        output=node["output"],
                        parameters=node["parameters"])

                ld.add_action(postSpawnNode)

########################################################################################


    return ld
