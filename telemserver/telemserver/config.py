import os
basedir = os.path.abspath(os.path.dirname(__file__))

class Config:
    SECRET_KEY = 'This is a secret key for you to change'
    DEBUG = False

class DevelopmentConfig(Config):
    DEBUG = True

class RobotConfig(Config):
    pass

config = DevelopmentConfig