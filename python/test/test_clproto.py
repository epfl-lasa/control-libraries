import unittest
import state_representation as sr
import clproto

class TestClprotoPackUnpack(unittest.TestCase):
    def test_pack_unpack(self):
        objects = [sr.CartesianState.Random("A", "B"), sr.JointState.Random("robot", ["j1", "j2", "j3"])]
        encoded_fields = [clproto.encode(objects[0], clproto.MessageType.CARTESIAN_STATE_MESSAGE),
                          clproto.encode(objects[1], clproto.MessageType.JOINT_STATE_MESSAGE)]
        packet = clproto.pack_fields(encoded_fields)

        received_fields = clproto.unpack_fields(packet)
        self.assertEqual(len(received_fields), len(encoded_fields))
        [self.assertTrue(clproto.is_valid(msg)) for msg in received_fields]
        decoded_objects = [clproto.decode(msg) for msg in received_fields]
        [self.assertIsInstance(new, type(ref)) for new, ref in zip(decoded_objects, objects)]

        # confirm cartesian state decoding
        self.assertEqual(objects[0].get_name(), decoded_objects[0].get_name())
        self.assertEqual(objects[0].get_reference_frame(), decoded_objects[0].get_reference_frame())
        self.assertAlmostEqual(sr.dist(objects[0], decoded_objects[0]), 0)

        # confirm joint state decoding
        self.assertEqual(objects[1].get_name(), decoded_objects[1].get_name())
        self.assertListEqual(objects[1].get_names(), decoded_objects[1].get_names())
        self.assertAlmostEqual(sr.dist(objects[1], decoded_objects[1]), 0)

class TestClprotoJSON(unittest.TestCase):
    def test_to_from_json(self):
        reference_object = sr.CartesianState.Random("A", "B")
        message_type = clproto.MessageType.CARTESIAN_STATE_MESSAGE
        msg = clproto.encode(reference_object, message_type)
        json = clproto.to_json(msg)
        msg2 = clproto.from_json(json)
        self.assertTrue(clproto.is_valid(msg2))
        self.assertEqual(clproto.check_message_type(msg2), message_type)
        decoded_object = clproto.decode(msg)
        self.assertIsInstance(decoded_object, type(reference_object))

        self.assertEqual(reference_object.get_name(), decoded_object.get_name())
        self.assertEqual(reference_object.get_reference_frame(), decoded_object.get_reference_frame())
        self.assertAlmostEqual(sr.dist(reference_object, decoded_object), 0)

class TestClprotoState(unittest.TestCase):
    def state_class_assertions(self, reference_object, message_type):
        object_type = type(reference_object)
        msg = clproto.encode(reference_object, message_type)
        self.assertTrue(clproto.is_valid(msg))
        self.assertEqual(clproto.check_message_type(msg), message_type)
        decoded_object = clproto.decode(msg)
        self.assertIsInstance(decoded_object, object_type)
        self.assertEqual(reference_object.get_name(), decoded_object.get_name())
        self.assertEqual(reference_object.get_type(), decoded_object.get_type())

    def test_encode_decode_state(self):
        self.state_class_assertions(sr.State(sr.StateType.STATE, "A"), clproto.MessageType.STATE_MESSAGE)

    def test_encode_decode_spatial_state(self):
        self.state_class_assertions(sr.SpatialState(sr.StateType.STATE, "A", "B"), clproto.MessageType.SPATIAL_STATE_MESSAGE)

class TestClprotoCartesian(unittest.TestCase):
    def cartesian_class_assertions(self, reference_object, message_type):
        object_type = type(reference_object)
        msg = clproto.encode(reference_object, message_type)
        self.assertTrue(clproto.is_valid(msg))
        self.assertEqual(clproto.check_message_type(msg), message_type)
        decoded_object = clproto.decode(msg)
        self.assertIsInstance(decoded_object, object_type)
        self.assertEqual(reference_object.get_name(), decoded_object.get_name())
        self.assertEqual(reference_object.get_reference_frame(), decoded_object.get_reference_frame())
        self.assertAlmostEqual(sr.dist(reference_object, decoded_object), 0)

    def test_encode_decode_cartesian_state(self):
        self.cartesian_class_assertions(sr.CartesianState.Random("A", "B"), clproto.MessageType.CARTESIAN_STATE_MESSAGE)

    def test_encode_decode_cartesian_pose(self):
        self.cartesian_class_assertions(sr.CartesianPose.Random("A", "B"), clproto.MessageType.CARTESIAN_POSE_MESSAGE)

    def test_encode_decode_cartesian_twist(self):
        self.cartesian_class_assertions(sr.CartesianTwist.Random("A", "B"), clproto.MessageType.CARTESIAN_TWIST_MESSAGE)

    def test_encode_decode_cartesian_wrench(self):
        self.cartesian_class_assertions(sr.CartesianWrench.Random("A", "B"), clproto.MessageType.CARTESIAN_WRENCH_MESSAGE)

class TestClprotoJoint(unittest.TestCase):
    def joint_class_assertions(self, reference_object, message_type):
        object_type = type(reference_object)
        msg = clproto.encode(reference_object, message_type)
        self.assertTrue(clproto.is_valid(msg))
        self.assertEqual(clproto.check_message_type(msg), message_type)
        decoded_object = clproto.decode(msg)
        self.assertIsInstance(decoded_object, object_type)
        self.assertEqual(reference_object.get_name(), decoded_object.get_name())
        self.assertListEqual(reference_object.get_names(), decoded_object.get_names())
        self.assertAlmostEqual(sr.dist(reference_object, decoded_object), 0)

    def test_encode_decode_joint_state(self):
        self.joint_class_assertions(sr.JointState.Random("robot", ["j1", "j2", "j3"]), clproto.MessageType.JOINT_STATE_MESSAGE)

    def test_encode_decode_joint_positions(self):
        self.joint_class_assertions(sr.JointPositions.Random("robot", ["j1", "j2", "j3"]), clproto.MessageType.JOINT_POSITIONS_MESSAGE)

    def test_encode_decode_joint_velocities(self):
        self.joint_class_assertions(sr.JointVelocities.Random("robot", ["j1", "j2", "j3"]), clproto.MessageType.JOINT_VELOCITIES_MESSAGE)

    def test_encode_decode_joint_accelerations(self):
        self.joint_class_assertions(sr.JointAccelerations.Random("robot", ["j1", "j2", "j3"]), clproto.MessageType.JOINT_ACCELERATIONS_MESSAGE)

    def test_encode_decode_joint_torques(self):
        self.joint_class_assertions(sr.JointTorques.Random("robot", ["j1", "j2", "j3"]), clproto.MessageType.JOINT_TORQUES_MESSAGE)

class TestClprotoJacobian(unittest.TestCase):
    def test_encode_decode_jacobian(self):
        obj1 = sr.Jacobian.Random("robot", ["j1", "j2", "j3"], "A", "B")
        msg = clproto.encode(obj1, clproto.MessageType.JACOBIAN_MESSAGE)
        self.assertTrue(clproto.is_valid(msg))
        self.assertEqual(clproto.check_message_type(msg), clproto.MessageType.JACOBIAN_MESSAGE)
        obj2 = clproto.decode(msg)
        self.assertIsInstance(obj2, sr.Jacobian)
        self.assertEqual(obj1.get_name(), obj2.get_name())
        self.assertEqual(obj1.get_frame(), obj2.get_frame())
        self.assertListEqual(obj1.get_joint_names(), obj2.get_joint_names())
        self.assertEqual(obj1.rows(), obj2.rows())
        self.assertEqual(obj1.cols(), obj2.cols())
        [self.assertAlmostEqual(x, y) for x, y in zip(obj1.data().flatten(), obj2.data().flatten())]

if __name__ == '__main__':
    unittest.main()