import os
import glob
from setuptools import find_packages, setup

package_name = 'ai_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*')),
        (os.path.join('share', package_name, 'weights'), glob.glob('weights/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bhavya-shah',
    maintainer_email='bhavya-shah@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'signal = ai_pipeline.signal:main',
            'camera_processor = ai_pipeline.camera_processor:main',
            'model_processor = ai_pipeline.model_processor:main',
            'streamer = ai_pipeline.streamer:main',
            'collator = ai_pipeline.collator:main',
        ],
    },
)
