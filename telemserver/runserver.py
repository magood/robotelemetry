"""
This script runs the telemserver application using a development server.
"""

from os import environ
from telemserver import app, socketio

if __name__ == '__main__':
    HOST = environ.get('SERVER_HOST', '0.0.0.0')
    try:
        PORT = int(environ.get('SERVER_PORT', '5555'))
    except ValueError:
        PORT = 5555
    socketio.run(app, HOST, PORT)
    #app.run(HOST, PORT)
