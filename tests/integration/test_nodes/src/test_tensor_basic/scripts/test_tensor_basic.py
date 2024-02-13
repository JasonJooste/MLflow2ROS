#!/usr/bin/env python

import unittest

import rospy
import tensor_basic_msgs.srv
from sklearn import datasets
from sklearn.model_selection import train_test_split


class TestListElements(unittest.TestCase):
    def test_response(self):
        rospy.init_node("test_tensor_basic", anonymous=False)

        # Load the Iris dataset
        X, y = datasets.load_iris(return_X_y=True)

        # Split the data into training and test sets
        _, X_test, _, _ = train_test_split(
            X, y, test_size=0.2, random_state=42
        )
        req = tensor_basic_msgs.srv.tensor_basicRequest(X_test.ravel())

        print("Waiting for service")
        rospy.wait_for_service("tensor_basic_service")

        logreg = rospy.ServiceProxy(
            "tensor_basic_service", tensor_basic_msgs.srv.tensor_basic
        )

        try:
            print("Sending request")
            predictions = logreg(req).tensor_basic_res
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        print("Received response")

        expected = ( # goddam it autoformatter
            1,
            0,
            2,
            1,
            1,
            0,
            1,
            2,
            1,
            1,
            2,
            0,
            0,
            0,
            0,
            1,
            2,
            1,
            1,
            2,
            0,
            2,
            0,
            2,
            2,
            2,
            2,
            2,
            0,
            0,
        )

        assert predictions == expected


if __name__ == "__main__":
    unittest.main()
