"""
The robot telementry web application package.
"""

import os
basedir = os.path.abspath(os.path.dirname(__file__))
from flask import Flask
from flask_socketio import SocketIO
from . import config

app = Flask(__name__)
app.config.from_object(config.config)
socketio = SocketIO(app, message_queue='redis://')

import telemserver.views
