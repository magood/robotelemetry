"""
Routes and views for the flask application.
"""

from datetime import datetime
from flask import render_template
from telemserver import app, socketio
from flask_socketio import send, emit
import time

@app.route('/')
@app.route('/home')
def home():
    """Renders the home page."""
    return render_template(
        'index.html',
        title='Home Page',
        year=datetime.now().year,
    )

@app.route('/test')
def test():
    """Renders a socketio test page."""
    return render_template(
        'test.html',
        title='SocketIO Test Page',
        year=datetime.now().year,
    )

@socketio.on('connect')
def test_connect():
    print('connected on server')
    socketio.emit('telem', {'data': 'Connected'})


@app.route('/contact')
def contact():
    """Renders the contact page."""
    return render_template(
        'contact.html',
        title='Contact',
        year=datetime.now().year,
        message='Your contact page.'
    )

@app.route('/about')
def about():
    """Renders the about page."""
    return render_template(
        'about.html',
        title='About',
        year=datetime.now().year,
        message='Your application description page.'
    )
