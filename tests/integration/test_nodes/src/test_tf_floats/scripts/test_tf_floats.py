#!/usr/bin/env python

import unittest

import rospy
import tf_floats_msgs.srv
import numpy as np


class TestListElements(unittest.TestCase):
    def test_response(self):
        rospy.init_node("test_tf_floats", anonymous=False)

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
        # ,
        # [
        # 2.7969000339508057,
        # 38,
        # 4.626707077026367,
        # 1.0789074897766113,
        # 2167,
        # 3.288315534591675,
        # 37.790000915527344,
        # -122.22000122070312
        # ],
        # [
        # 3.2249999046325684,
        # 44,
        # 5.854984760284424,
        # 1.2054380178451538,
        # 946,
        # 2.858006000518799,
        # 37.75,
        # -122.1500015258789
        # ]

        X_test = np.array(X_test)

        req = tf_floats_msgs.srv.tf_floatsRequest(X_test.ravel())

        print("Waiting for service")
        rospy.wait_for_service("tf_floats_service")

        logreg = rospy.ServiceProxy(
            "tf_floats_service", tf_floats_msgs.srv.tf_floats
        )

        try:
            print("Sending request")
            predictions = logreg(req).tf_floats_res
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        print("Received response")

        expected = (2.2163126468658447,)

        assert predictions == expected


if __name__ == "__main__":
    unittest.main()
