#!/usr/bin/env python

import unittest

import rospy
import tf_floats_model_msgs.srv
import numpy as np


class TestListElements(unittest.TestCase):
    def test_response(self):
        rospy.init_node("tf_floats_node", anonymous=False)

        X_test = [
        4.020199775695801,
        43,
        4.794676780700684,
        1.031685709953308,
        2139,
        2.711026668548584,
        37.720001220703125,
        -122.16999816894531
        ]

        X_test = np.array(X_test)

        req = tf_floats_model_msgs.srv.tf_floats_modelRequest(X_test.ravel())

        print("Waiting for service")
        rospy.wait_for_service("tf_floats_model_service")

        logreg = rospy.ServiceProxy(
            "tf_floats_model_service", tf_floats_model_msgs.srv.tf_floats_model
        )

        try:
            print("Sending request")
            predictions = logreg(req).tf_floats_model_res
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        print("Received response")

        expected = (2.2163126468658447,)

        assert predictions == expected


if __name__ == "__main__":
    unittest.main()
