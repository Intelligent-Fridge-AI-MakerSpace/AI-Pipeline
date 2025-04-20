from launch import LaunchDescription
from launch_ros.actions import Node

def create_pipeline_nodes(camera_id):
    namespace = f'camera_{camera_id}'
    width = 640
    height = 640
    fps = 30
    ip_address = '192.168.209.23'
    port = 8554
    stream_id = namespace
    model_name = 'yolo' if int(camera_id) % 2 == 0 else 'barcode'

    camera_processor_node = Node(
        package='ai_pipeline',
        executable='camera_processor',
        name='camera_processor',
        namespace=namespace,
        output='screen',
        parameters=[
            {'camera_id': camera_id},
            {'width': width},
            {'height': height},
        ],
        remappings=[
            ('camera', f'/{namespace}/camera'),
        ],
    )

    model_processor_node = Node(
        package='ai_pipeline',
        executable='model_processor',
        name='model_processor',
        namespace=namespace,
        output='screen',
        parameters=[
            {'camera_id': camera_id},
            {'model_name': model_name},
            {'conf': 0.5},
            {'iou': 0.5},
            {'use_gpu': True},
            {'iou_threshold': 0.9},
            {'tracker_type': 'median_flow'},
        ],
        remappings=[
            ('camera', f'/{namespace}/camera'),
            ('stream', f'/{namespace}/stream'),
            ('result', f'/{namespace}/result'),
        ],
    )

    streamer_node = Node(
        package='ai_pipeline',
        executable='streamer',
        name='streamer',
        namespace=namespace,
        output='screen',
        parameters=[
            {'camera_id': camera_id},
            {'width': width},
            {'height': height},
            {'fps': fps},
            {'ip_address': ip_address},
            {'port': port},
            {'stream_id': stream_id},
        ],
        remappings=[
            ('stream', f'/{namespace}/stream'),
        ],
    )

    return [camera_processor_node, model_processor_node, streamer_node]

def generate_launch_description():
    camera_ids = [0, 1]

    signal_node = Node(
        package='ai_pipeline',
        executable='signal',
        name='signal',
        output='screen',
    )

    collator_node = Node(
        package='ai_pipeline',
        executable='collator',
        name='collator',
        output='screen',
        parameters=[
            {'camera_ids': camera_ids}
            ],
        remappings=[],
    )

    pipeline_nodes = []
    for camera_id in camera_ids:
        pipeline_nodes = pipeline_nodes + create_pipeline_nodes(camera_id)

    return LaunchDescription([
        signal_node,
        *pipeline_nodes,
        collator_node,
    ])
