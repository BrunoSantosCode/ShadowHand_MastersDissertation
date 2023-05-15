#!/usr/bin/env python3

from itertools import count
from operator import index
import rospy
from openpose_mujoco.msg import Pixel, BodyPart, Person, Frame

def openpose_publisher():
    pub = rospy.Publisher('frame_topic', Frame, queue_size=1)
    rospy.init_node('openpose_publisher', anonymous=True)

    msg = Frame()
    msg.header.frame_id = '01'

    person = Person()

    wrist = BodyPart()
    wrist.point.x, wrist.point.y, wrist.point.z = 3.0, 1.0, 0.0
    person.rightHandParts.append(wrist)

    thumb1 = BodyPart()
    thumb1.point.x, thumb1.point.y, thumb1.point.z = 4.0, 2.0, 0.0
    thumb2 = BodyPart()
    thumb2.point.x, thumb2.point.y, thumb2.point.z = 5.0, 3.0, 0.0
    thumb3 = BodyPart()
    thumb3.point.x, thumb3.point.y, thumb3.point.z = 5.0, 4.0, 0.0
    thumb4 = BodyPart()
    thumb4.point.x, thumb4.point.y, thumb4.point.z = 5.0, 5.0, 0.0
    person.rightHandParts.append(thumb1)
    person.rightHandParts.append(thumb2)
    person.rightHandParts.append(thumb3)
    person.rightHandParts.append(thumb4)

    index5 = BodyPart()
    index5.point.x, index5.point.y, index5.point.z = 4.0, 4.0, 0.0
    index6 = BodyPart()
    index6.point.x, index6.point.y, index6.point.z = 4.0, 5.0, 0.0
    index7 = BodyPart()
    index7.point.x, index7.point.y, index7.point.z = 4.0, 6.0, 0.0
    index8 = BodyPart()
    index8.point.x, index8.point.y, index8.point.z = 4.0, 7.0, 0.0
    person.rightHandParts.append(index5)
    person.rightHandParts.append(index6)
    person.rightHandParts.append(index7)
    person.rightHandParts.append(index8)

    middle9 = BodyPart()
    middle9.point.x, middle9.point.y, middle9.point.z = 3.0, 4.0, 0.0
    middle10 = BodyPart()
    middle10.point.x, middle10.point.y, middle10.point.z = 3.0, 5.0, 0.0
    middle11 = BodyPart()
    middle11.point.x, middle11.point.y, middle11.point.z = 3.0, 6.0, 0.0
    middle12 = BodyPart()
    middle12.point.x, middle12.point.y, middle12.point.z = 3.0, 7.0, 0.0
    person.rightHandParts.append(middle9)
    person.rightHandParts.append(middle10)
    person.rightHandParts.append(middle11)
    person.rightHandParts.append(middle12)

    ring13 = BodyPart()
    ring13.point.x, ring13.point.y, ring13.point.z = 2.0, 4.0, 0.0
    ring14 = BodyPart()
    ring14.point.x, ring14.point.y, ring14.point.z = 2.0, 5.0, 0.0
    ring15 = BodyPart()
    ring15.point.x, ring15.point.y, ring15.point.z = 2.0, 6.0, 0.0
    ring16 = BodyPart()
    ring16.point.x, ring16.point.y, ring16.point.z = 2.0, 7.0, 0.0
    person.rightHandParts.append(ring13)
    person.rightHandParts.append(ring14)
    person.rightHandParts.append(ring15)
    person.rightHandParts.append(ring16)

    little17 = BodyPart()
    little17.point.x, little17.point.y, little17.point.z = 1.0, 4.0, 0.0
    little18 = BodyPart()
    little18.point.x, little18.point.y, little18.point.z = 1.0, 5.0, 0.0
    little19 = BodyPart()
    little19.point.x, little19.point.y, little19.point.z = 1.0, 6.0, 0.0
    little20 = BodyPart()
    little20.point.x, little20.point.y, little20.point.z = 1.0, 7.0, 0.0
    person.rightHandParts.append(little17)
    person.rightHandParts.append(little18)
    person.rightHandParts.append(little19)
    person.rightHandParts.append(little20)

    msg.persons.append(person)


    pub.publish(msg)
    print('Msg published!')

if __name__ == '__main__':
    try:
        openpose_publisher()
    except rospy.ROSInterruptException:
        pass
