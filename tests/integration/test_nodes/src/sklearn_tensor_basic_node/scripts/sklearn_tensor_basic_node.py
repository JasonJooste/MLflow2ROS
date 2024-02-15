#!/usr/bin/env python

import unittest

import rospy
import sklearn_tensor_basic_model_msgs.srv
from sklearn import datasets
from sklearn.model_selection import train_test_split


class TestListElements(unittest.TestCase):
    def test_response(self):
        rospy.init_node("sklearn_tensor_basic_node", anonymous=False)

        # Load the Iris dataset
        X, y = datasets.load_iris(return_X_y=True)

        # Split the data into training and test sets
        _, X_test, _, _ = train_test_split(X, y, test_size=0.2, random_state=42)
        req = sklearn_tensor_basic_model_msgs.srv.sklearn_tensor_basic_modelRequest(
            X_test.ravel()
        )

        print("Waiting for service")
        rospy.wait_for_service("sklearn_tensor_basic_model_service")

        logreg = rospy.ServiceProxy(
            "sklearn_tensor_basic_model_service",
            sklearn_tensor_basic_model_msgs.srv.sklearn_tensor_basic_model,
        )

        try:
            print("Sending request")
            predictions = logreg(req).sklearn_tensor_basic_model_res
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        print("Received response")

        expected = (  # goddam it autoformatter
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
