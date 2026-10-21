"""The discarded surveyed-board solve must never qualify TCP planning."""
import unittest
from unloading_sim.eco65.perception import signed, require_tcp_calibration


class TCPBindingTests(unittest.TestCase):
    def test_surveyed_board_pass_is_ineligible(self):
        record=signed(dict(status='PASS',calibration_method='SURVEYED_BOARD_ONLY_DIAGNOSTIC',
            validation_rms_px=.2,T_link1_optical=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]))
        with self.assertRaisesRegex(ValueError,'TCP_REFERENCED_CALIBRATION_REQUIRED'):
            require_tcp_calibration(record)

    def test_tcp_name_without_samples_is_ineligible(self):
        record=signed(dict(status='PASS',calibration_method='TCP_FK_REFERENCED_CAMERA_EXTRINSICS'))
        with self.assertRaisesRegex(ValueError,'TCP_REFERENCED_CALIBRATION_REQUIRED'):
            require_tcp_calibration(record)

    def test_changed_record_rejected(self):
        record=signed(dict(status='FAIL')); record['status']='PASS'
        with self.assertRaisesRegex(ValueError,'PERCEPTION_RECORD_CHANGED'):
            require_tcp_calibration(record)


if __name__=='__main__':unittest.main()
