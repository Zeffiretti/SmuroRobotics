import setuptools

# with open("README.md", "r", encoding="utf-8") as fh:
#     long_description = fh.read()

setuptools.setup(
    name='smuro_robotics',
    version='0.0.2',
    author='Zeffiretti Hiesh',
    author_email='hiesh@mail.com',
    description='Smuro Robotics Caculation Package',
    long_description="long_description",
    long_description_content_type="text",
    url='https://github.com/zeffiretti/SmuroRobotics',
    project_urls = {
        "Bug Tracker": "https://github.com/zeffiretti/SmuroRobotics/issues"
    },
    license='GNU Lesser General Public License v2.1',
    packages=['smuro_robotics'],
    install_requires=['numpy','modern_robotics'],
)