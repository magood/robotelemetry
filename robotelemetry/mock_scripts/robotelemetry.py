from flask_socketio import SocketIO
import time

socketio = SocketIO(message_queue='redis://')

if __name__ == '__main__':
    for i in range(15):
        socketio.emit('telem', {'data': i})
        print(i)
        time.sleep(2)
