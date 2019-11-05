import logging
import threading

import rospy
from std_msgs.msg import String

from flask import Flask, render_template
from flask_ask import Ask, statement, question, session

threading.Thread(target=lambda: rospy.init_node('test_code', disable_signals=True)).start()
pub = rospy.Publisher('test_pub', String, queue_size=1)

app = Flask(__name__)
ask = Ask(app, "/")
logging.getLogger("flask_ask").setLevel(logging.DEBUG)

@ask.launch
def launch():
    welcome_msg = render_template('welcome')
    return question(welcome_msg)

@ask.intent('HelloIntent')
def hello(firstname):
    text = render_template('hello', firstname=firstname)
    pub.publish(firstname)
    return statement(text).simple_card('Hello', text)

if __name__ == '__main__':
    app.run(debug=True)
