from setuptools import setup

package_name = 'yolov8_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ✅ 아래 줄을 추가해서 launch 파일 복사되도록
        ('share/' + package_name + '/launch', ['launch/yolo_system_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hs',
    maintainer_email='your@email.com',
    description='YOLOv8 with Camera launcher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
