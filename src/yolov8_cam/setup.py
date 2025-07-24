from setuptools import setup

package_name = 'yolov8_cam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Webcam image publisher for YOLOv8',
    license='MIT',
    entry_points={
        'console_scripts': [
            'webcam_publisher = yolov8_cam.webcam_publisher:main'
        ],
    },
)
