# AGX Dynamics imports
import agx
import agxCollide
import agxSDK
import agxModel
import agxOSG
import agxRender
import agxUtil
import agxIO
import agxCable
import agxTerrain

from agxPythonModules.utils.environment import simulation, init_app, application, root
from agxPythonModules.robots.panda import Panda  # noqa

from segway import Segway

import math
from math import pi, radians, sin, cos, copysign
import numpy as np

from collections import namedtuple

def disable_collisions(panda, linkA, linkB):
    simulation().getSpace().setEnablePair(linkA, linkB, False)
    for lA in panda.assembly.getRigidBody(linkA).getGeometries(): 
      for lB in panda.assembly.getRigidBody(linkB).getGeometries():
        simulation().getSpace().setEnableCollisions(lA, lB, False)


# Build robot scene
def buildScene():

    #add ground
    ground = agxCollide.Geometry(agxCollide.Box(30, 50, 0.1))
    ground.setPosition(agx.Vec3(0, 0, -0.3))
    ground.setName("ground")
    simulation().add(ground)
    ground_node = agxOSG.createVisual(ground, root())
    agxOSG.setDiffuseColor(ground_node, agxRender.Color.Black())
    
    groundGeometries = []

    def createGroundGeometry(**kwargs) -> agxCollide.Geometry:
        geometry = agxCollide.Geometry(kwargs['shape'])
        geometry.setTransform(kwargs.get('transform', agx.AffineMatrix4x4()))
        simulation().add(geometry)
        agxOSG.setDiffuseColor(agxOSG.createVisual(geometry, root(), 1.5),
                               kwargs.get('color', agxRender.Color.Brown()))
        groundGeometries.append(geometry)
        return geometry

    def buildHump(alpha,a,h,offset,offset_x):
      createGroundGeometry(shape=agxCollide.Box(a, 1, h),
                         transform=agx.AffineMatrix4x4.rotate(radians(alpha), agx.Vec3.X_AXIS()) *
                         agx.AffineMatrix4x4.translate(offset_x,offset, -0.4),
                         color=agxRender.Color.DarkCyan())
      createGroundGeometry(shape=agxCollide.Box(a, 1, h),
                         transform=agx.AffineMatrix4x4.rotate(radians(-alpha), agx.Vec3.X_AXIS()) *
                         agx.AffineMatrix4x4.translate(offset_x,offset+cos(radians(alpha))*(2-h),-0.4),
                         color=agxRender.Color.DarkCyan())

    buildHump(10,3,0.1,5,0)
#    buildHump(20,1,0.1,4.5)
#    buildHump(30,1,0.1,8)
    buildHump(15,3,0.1,13,0)
#    buildHump(20,3,0.1,30,0)
    buildHump(10,3,0.1,35,2.5)

    # The robot
    #
    seg = Segway(sim=simulation(), disable_self_collision=True)
    #panda = Panda(simulation(), use_tool=True, disable_self_collision=False)

    # sets maximum torque
    #maximum_torque = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
    #for tau, i in zip(maximum_torque, range(7)):
       #print("Setting tau_max ",tau," for motor ",i)
    #    panda.assembly.getConstraint1DOF("panda_joint"+str(i+1)).getMotor1D().setForceRange(-tau, tau)

#    disable_collisions(panda,"panda_link0","panda_link1")
#to disable all self-collisions
#    simulation().getSpace().setEnablePair(panda.assembly.getName(), panda.assembly.getName(), False)
#    print("robot ", panda.assembly.getName())
#    print("ground ",ground.getName())
    
    #set contact materials
    groundMaterial = agx.Material('ground')
    wheelMaterial = agx.Material('wheel')

    wheelGroundContactMaterial = simulation().getMaterialManager().getOrCreateContactMaterial(wheelMaterial,
                                                                                              groundMaterial)  # type: agx.ContactMaterial
    wheelGroundContactMaterial.setRestitution(0)
    wheelGroundContactMaterial.setFrictionCoefficient(1000.0, agx.ContactMaterial.PRIMARY_DIRECTION)
    wheelGroundContactMaterial.setFrictionCoefficient(10.0, agx.ContactMaterial.SECONDARY_DIRECTION)
    wheelGroundContactMaterial.setSurfaceViscosity(1.0E-7, agx.ContactMaterial.PRIMARY_DIRECTION)
    wheelGroundContactMaterial.setSurfaceViscosity(8.0E-6, agx.ContactMaterial.SECONDARY_DIRECTION)

    wheelGroundContactMaterial.setFrictionModel(agx.ConstantNormalForceOrientedBoxFrictionModel(10 * (seg.chassis.getMassProperties().getMass()),
                                                                                                seg.chassis.getFrame(),
                                                                                                agx.Vec3.X_AXIS(),
                                                                                                agx.FrictionModel.DIRECT,
                                                                                                False))
    ground.setMaterial(groundMaterial)
    for groundGeometry in groundGeometries:
        groundGeometry.setMaterial(groundMaterial)
    for wheel in seg.wheels:
        wheel.getGeometries()[0].setMaterial(wheelMaterial)
  

    seg.enable_motors(True)
#    joint_names = ec.get_joint_names()
#    joint_positions = ec.get_joint_velocities()
#    for jp,jn in zip(joint_positions, joint_names):
#      print ("joint",jp,"at",jn)

    #joint_positions[1] = 0.5
#    ec.set_joint_velocities(joint_positions)

    agxIO.writeFile("segway_scene.agx",simulation())

def onAppInitialized(app: agxOSG.ExampleApplication):
    cameraData = app.getCameraData()
    cameraData.eye = agx.Vec3(-7.5, -0.2, 1.7)
    cameraData.center = agx.Vec3(-0.3101, 0.0829, 0.1390)
    cameraData.up = agx.Vec3(0.214, -0.0112, 0.9767)
    cameraData.nearClippingPlane = 0.1
    cameraData.farClippingPlane = 5000
    app.applyCameraData(cameraData)

    app.getSceneDecorator().setEnableShadows(True)
    app.getSceneDecorator().setShadowMethod(agxOSG.SceneDecorator.SOFT_SHADOWMAP)

    def toVec3(v):
        return agx.Vec3(v[0], v[1], v[2])
    app.getSceneDecorator().setBackgroundColor(toVec3(agxRender.Color.SkyBlue()), toVec3(agxRender.Color.DodgerBlue()))


# Entry point when this script is started with python executable
init = init_app(
    name=__name__,
    scenes=[(buildScene, "1")],
    autoStepping=False,  # Default: False
    onInitialized=onAppInitialized,
    onShutdown=lambda app: print("App successfully shut down."),
)
