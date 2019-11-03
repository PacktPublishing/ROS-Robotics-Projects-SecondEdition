from setuptools import setup
setup(
    name='ros2_talker',
    version='0.0.0',
    packages=[],
    py_modules=['talker'],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ramkumar Gandhinathan',
    author_email='ram651991@gmail.com',
    maintainer='Ramkumar Gandhinathan',
    maintainer_email='ram651991@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Example to explain ROS-2',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'talker = talker:talker_main'
        ],
    },
)
