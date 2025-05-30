#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import xml.etree.ElementTree as ET
import re
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class SdfToUrdfPublisher(Node):
    def __init__(self):
        super().__init__('sdf_to_urdf_publisher')
        
        # Parameters
        self.declare_parameter('namespace', 'drone0')
        self.declare_parameter('mesh_base_url', 'http://localhost:8000')
        
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.mesh_base_url = self.get_parameter('mesh_base_url').get_parameter_value().string_value
        
        # QoS profile for latched topics (transient local)
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        
        # Subscriber to SDF robot description with transient local QoS
        self.sdf_subscription = self.create_subscription(
            String,
            f'/{self.namespace}/robot_description',
            self.sdf_callback,
            qos_profile
        )
        
        # Publisher for URDF robot description with transient local QoS
        self.urdf_publisher = self.create_publisher(
            String,
            f'/{self.namespace}/robot_description_urdf',
            qos_profile
        )
        
        self.get_logger().info(f'SDF to URDF converter started for namespace: {self.namespace}')
        self.get_logger().info('Waiting for robot description...')

    def sdf_callback(self, msg):
        """Convert SDF to URDF and publish"""
        try:
            self.get_logger().info('Received SDF robot description, converting to URDF...')
            urdf_content = self.convert_sdf_to_urdf(msg.data)
            urdf_msg = String()
            urdf_msg.data = urdf_content
            
            # Publish URDF
            self.urdf_publisher.publish(urdf_msg)
            
            self.get_logger().info('Published URDF robot description')
            
            # Log some info about the conversion
            self.get_logger().info(f'URDF contains {urdf_content.count("<link")} links and {urdf_content.count("<joint")} joints')
            
        except Exception as e:
            self.get_logger().error(f'Failed to convert SDF to URDF: {str(e)}')

    def convert_sdf_to_urdf(self, sdf_content):
        """Convert SDF content to URDF format"""
        # Parse SDF XML
        root = ET.fromstring(sdf_content)
        
        # Create URDF root element
        urdf_root = ET.Element('robot')
        urdf_root.set('name', self.namespace)
        
        # Find the model element in SDF
        model = root.find('.//model')
        if model is None:
            raise ValueError("No model found in SDF")
        
        # Convert links
        for sdf_link in model.findall('link'):
            urdf_link = self.convert_link(sdf_link)
            if urdf_link is not None:
                urdf_root.append(urdf_link)
        
        # Convert joints
        for sdf_joint in model.findall('joint'):
            urdf_joint = self.convert_joint(sdf_joint)
            if urdf_joint is not None:
                urdf_root.append(urdf_joint)
        
        # Convert to string with proper XML declaration
        urdf_str = ET.tostring(urdf_root, encoding='unicode')
        return f'<?xml version="1.0"?>\n{urdf_str}'

    def convert_link(self, sdf_link):
        """Convert SDF link to URDF link"""
        link_name = sdf_link.get('name')
        if not link_name:
            return None
        
        urdf_link = ET.Element('link')
        urdf_link.set('name', link_name)
        
        # Convert inertial properties
        sdf_inertial = sdf_link.find('inertial')
        if sdf_inertial is not None:
            urdf_inertial = self.convert_inertial(sdf_inertial)
            urdf_link.append(urdf_inertial)
        
        # Convert visual elements
        for sdf_visual in sdf_link.findall('visual'):
            urdf_visual = self.convert_visual(sdf_visual)
            if urdf_visual is not None:
                urdf_link.append(urdf_visual)
        
        # Convert collision elements
        for sdf_collision in sdf_link.findall('collision'):
            urdf_collision = self.convert_collision(sdf_collision)
            if urdf_collision is not None:
                urdf_link.append(urdf_collision)
        
        return urdf_link

    def convert_inertial(self, sdf_inertial):
        """Convert SDF inertial to URDF inertial"""
        urdf_inertial = ET.Element('inertial')
        
        # Origin (pose)
        sdf_pose = sdf_inertial.find('pose')
        if sdf_pose is not None:
            pose_values = sdf_pose.text.strip().split()
            if len(pose_values) >= 6:
                origin = ET.SubElement(urdf_inertial, 'origin')
                origin.set('xyz', f"{pose_values[0]} {pose_values[1]} {pose_values[2]}")
                origin.set('rpy', f"{pose_values[3]} {pose_values[4]} {pose_values[5]}")
        
        # Mass
        sdf_mass = sdf_inertial.find('mass')
        if sdf_mass is not None:
            mass = ET.SubElement(urdf_inertial, 'mass')
            mass.set('value', sdf_mass.text.strip())
        
        # Inertia
        sdf_inertia = sdf_inertial.find('inertia')
        if sdf_inertia is not None:
            inertia = ET.SubElement(urdf_inertial, 'inertia')
            for attr in ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']:
                elem = sdf_inertia.find(attr)
                if elem is not None:
                    inertia.set(attr, elem.text.strip())
        
        return urdf_inertial

    def convert_visual(self, sdf_visual):
        """Convert SDF visual to URDF visual"""
        urdf_visual = ET.Element('visual')
        
        # Name
        visual_name = sdf_visual.get('name')
        if visual_name:
            urdf_visual.set('name', visual_name)
        
        # Origin (pose)
        sdf_pose = sdf_visual.find('pose')
        if sdf_pose is not None:
            pose_values = sdf_pose.text.strip().split()
            if len(pose_values) >= 6:
                origin = ET.SubElement(urdf_visual, 'origin')
                origin.set('xyz', f"{pose_values[0]} {pose_values[1]} {pose_values[2]}")
                origin.set('rpy', f"{pose_values[3]} {pose_values[4]} {pose_values[5]}")
        
        # Geometry
        sdf_geometry = sdf_visual.find('geometry')
        if sdf_geometry is not None:
            urdf_geometry = self.convert_geometry(sdf_geometry)
            urdf_visual.append(urdf_geometry)
        
        return urdf_visual

    def convert_collision(self, sdf_collision):
        """Convert SDF collision to URDF collision"""
        urdf_collision = ET.Element('collision')
        
        # Name
        collision_name = sdf_collision.get('name')
        if collision_name:
            urdf_collision.set('name', collision_name)
        
        # Origin (pose)
        sdf_pose = sdf_collision.find('pose')
        if sdf_pose is not None:
            pose_values = sdf_pose.text.strip().split()
            if len(pose_values) >= 6:
                origin = ET.SubElement(urdf_collision, 'origin')
                origin.set('xyz', f"{pose_values[0]} {pose_values[1]} {pose_values[2]}")
                origin.set('rpy', f"{pose_values[3]} {pose_values[4]} {pose_values[5]}")
        
        # Geometry
        sdf_geometry = sdf_collision.find('geometry')
        if sdf_geometry is not None:
            urdf_geometry = self.convert_geometry(sdf_geometry)
            urdf_collision.append(urdf_geometry)
        
        return urdf_collision

    def convert_geometry(self, sdf_geometry):
        """Convert SDF geometry to URDF geometry"""
        urdf_geometry = ET.Element('geometry')
        
        # Check for mesh
        sdf_mesh = sdf_geometry.find('mesh')
        if sdf_mesh is not None:
            urdf_mesh = ET.SubElement(urdf_geometry, 'mesh')
            
            # Convert URI
            sdf_uri = sdf_mesh.find('uri')
            if sdf_uri is not None:
                uri_text = sdf_uri.text.strip()
                # Convert package:// URI to HTTP URL
                if uri_text.startswith('package://'):
                    # Extract the path after package://
                    package_path = uri_text.replace('package://', '')
                    # Convert to HTTP URL for ros3djs compatibility
                    # Add /share/ to match the install directory structure
                    if package_path.startswith('as2_gazebo_assets/'):
                        package_path = package_path.replace('as2_gazebo_assets/', 'as2_gazebo_assets/share/as2_gazebo_assets/', 1)
                    http_url = f"{self.mesh_base_url}/{package_path}"
                    urdf_mesh.set('filename', http_url)
                    self.get_logger().info(f'Converted mesh URI: {uri_text} -> {http_url}')
                else:
                    urdf_mesh.set('filename', uri_text)
            
            # Scale
            sdf_scale = sdf_mesh.find('scale')
            if sdf_scale is not None:
                urdf_mesh.set('scale', sdf_scale.text.strip())
        
        # Check for box
        sdf_box = sdf_geometry.find('box')
        if sdf_box is not None:
            urdf_box = ET.SubElement(urdf_geometry, 'box')
            sdf_size = sdf_box.find('size')
            if sdf_size is not None:
                urdf_box.set('size', sdf_size.text.strip())
        
        # Check for cylinder
        sdf_cylinder = sdf_geometry.find('cylinder')
        if sdf_cylinder is not None:
            urdf_cylinder = ET.SubElement(urdf_geometry, 'cylinder')
            sdf_radius = sdf_cylinder.find('radius')
            sdf_length = sdf_cylinder.find('length')
            if sdf_radius is not None:
                urdf_cylinder.set('radius', sdf_radius.text.strip())
            if sdf_length is not None:
                urdf_cylinder.set('length', sdf_length.text.strip())
        
        # Check for sphere
        sdf_sphere = sdf_geometry.find('sphere')
        if sdf_sphere is not None:
            urdf_sphere = ET.SubElement(urdf_geometry, 'sphere')
            sdf_radius = sdf_sphere.find('radius')
            if sdf_radius is not None:
                urdf_sphere.set('radius', sdf_radius.text.strip())
        
        return urdf_geometry

    def convert_joint(self, sdf_joint):
        """Convert SDF joint to URDF joint"""
        joint_name = sdf_joint.get('name')
        joint_type = sdf_joint.get('type')
        
        if not joint_name or not joint_type:
            return None
        
        urdf_joint = ET.Element('joint')
        urdf_joint.set('name', joint_name)
        urdf_joint.set('type', joint_type)
        
        # Parent and child
        sdf_parent = sdf_joint.find('parent')
        sdf_child = sdf_joint.find('child')
        
        if sdf_parent is not None:
            parent = ET.SubElement(urdf_joint, 'parent')
            parent.set('link', sdf_parent.text.strip())
        
        if sdf_child is not None:
            child = ET.SubElement(urdf_joint, 'child')
            child.set('link', sdf_child.text.strip())
        
        # Origin (pose)
        sdf_pose = sdf_joint.find('pose')
        if sdf_pose is not None:
            pose_values = sdf_pose.text.strip().split()
            if len(pose_values) >= 6:
                origin = ET.SubElement(urdf_joint, 'origin')
                origin.set('xyz', f"{pose_values[0]} {pose_values[1]} {pose_values[2]}")
                origin.set('rpy', f"{pose_values[3]} {pose_values[4]} {pose_values[5]}")
        
        # Axis
        sdf_axis = sdf_joint.find('axis')
        if sdf_axis is not None:
            sdf_xyz = sdf_axis.find('xyz')
            if sdf_xyz is not None:
                axis = ET.SubElement(urdf_joint, 'axis')
                axis.set('xyz', sdf_xyz.text.strip())
        
        return urdf_joint


def main(args=None):
    rclpy.init(args=args)
    node = SdfToUrdfPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 