import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Dapatkan path instalasi package MAVROS
    mavros_share_dir = get_package_share_directory('mavros')

    # Siapkan LaunchConfigurations untuk menampung nilai argumen
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    tgt_system = LaunchConfiguration('tgt_system')
    tgt_component = LaunchConfiguration('tgt_component')
    log_output = LaunchConfiguration('log_output')
    fcu_protocol = LaunchConfiguration('fcu_protocol')
    respawn_mavros = LaunchConfiguration('respawn_mavros')
    namespace = LaunchConfiguration('namespace')

    # Deklarasi argumen yang bisa diubah saat menjalankan perintah ros2 launch
    declare_fcu_url = DeclareLaunchArgument(
        'fcu_url', default_value='serial:///dev/ttyACM0:115200',
        description='Koneksi serial ke Flight Controller')
    
    declare_gcs_url = DeclareLaunchArgument(
        'gcs_url', default_value='',
        description='Koneksi ke Ground Control Station')
    
    declare_tgt_system = DeclareLaunchArgument(
        'tgt_system', default_value='1',
        description='Target system ID')
    
    declare_tgt_component = DeclareLaunchArgument(
        'tgt_component', default_value='1',
        description='Target component ID')
    
    declare_log_output = DeclareLaunchArgument(
        'log_output', default_value='screen',
        description='Output log')
    
    declare_fcu_protocol = DeclareLaunchArgument(
        'fcu_protocol', default_value='v2.0',
        description='MAVLink protocol version')
    
    declare_respawn_mavros = DeclareLaunchArgument(
        'respawn_mavros', default_value='false',
        description='Restart otomatis jika MAVROS crash')
    
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='mavros',
        description='Namespace untuk node MAVROS')

    # Include file node.launch milik MAVROS dan teruskan parameter konfigurasi APM
    include_mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(mavros_share_dir, 'launch', 'node.launch')
        ),
        launch_arguments={
            'pluginlists_yaml': os.path.join(mavros_share_dir, 'launch', 'apm_pluginlists.yaml'),
            'config_yaml': os.path.join(mavros_share_dir, 'launch', 'apm_config.yaml'),
            'fcu_url': fcu_url,
            'gcs_url': gcs_url,
            'tgt_system': tgt_system,
            'tgt_component': tgt_component,
            'fcu_protocol': fcu_protocol,
            'respawn_mavros': respawn_mavros,
            'namespace': namespace
        }.items()
    )

    # Kembalikan daftar semua aksi yang akan dieksekusi oleh ROS 2
    return LaunchDescription([
        declare_fcu_url,
        declare_gcs_url,
        declare_tgt_system,
        declare_tgt_component,
        declare_log_output,
        declare_fcu_protocol,
        declare_respawn_mavros,
        declare_namespace,
        include_mavros_node
    ])