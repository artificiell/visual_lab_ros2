from setuptools import find_packages, setup

package_name = 'speech_to_text'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andreas Persson',
    maintainer_email='andreas.persson@oru.se',
    description='The speech to text package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transcribe = speech_to_text.transcribe:main',
            'keyword = speech_to_text.keyword:main'
        ],
    },
)
