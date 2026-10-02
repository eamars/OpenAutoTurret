"""Unit tests for the read-only audit utilities, not plant qualification."""
import unittest
import numpy as np
from audit_evidence import AuditError, compare, validate_observations, close


def fixture():
    return dict(t=np.array([0., .01, .02]), encoder_q=np.array([0., .1, .2]),
        gyro_v=np.array([10., 999., 10.]), decoded_current=np.array([.2,.2,.2]),
        q_new=np.array([True,True,True]),v_new=np.array([True,False,True]),
        current_new=np.array([True,True,True]),tx_t=np.array([-.1, .01]),tx_A=np.array([0.,.2]))


class AuditTests(unittest.TestCase):
    def test_native_mask_not_repeated_gyro(self):
        o=fixture()
        p=dict(encoder_prediction_rad=np.array([0.,.1,.2]),
               gyro_prediction_rad_s=np.array([10.,10.]),
               reported_current_prediction_A=np.array([.2,.2,.2]))
        validate_observations(o,'fixture',('encoder_q','gyro_v','decoded_current'))
        self.assertEqual(compare(o,p,('encoder_q','gyro_v','decoded_current'))['v']['rms'],0.)
    def test_reject_time_reset(self):
        o=fixture();o['t'][2]=-.1
        with self.assertRaises(AuditError): validate_observations(o,'fixture',('encoder_q','gyro_v','decoded_current'))
    def test_reject_no_input_prehistory(self):
        o=fixture();o['tx_t']=np.array([.001,.01])
        with self.assertRaises(AuditError): validate_observations(o,'fixture',('encoder_q','gyro_v','decoded_current'))
    def test_reject_integer_mask(self):
        o=fixture();o['v_new']=o['v_new'].astype(int)
        with self.assertRaises(AuditError): validate_observations(o,'fixture',('encoder_q','gyro_v','decoded_current'))
    def test_reject_nonfinite_native_data(self):
        o=fixture();o['encoder_q'][1]=np.nan
        with self.assertRaises(AuditError): validate_observations(o,'fixture',('encoder_q','gyro_v','decoded_current'))
    def test_reject_prediction_shape(self):
        p=dict(encoder_prediction_rad=np.array([0.,.1]),gyro_prediction_rad_s=np.array([10.,10.]),
               reported_current_prediction_A=np.array([.2,.2,.2]))
        with self.assertRaises(AuditError):compare(fixture(),p,('encoder_q','gyro_v','decoded_current'))
    def test_metrics_comparison_rejects_mismatch(self):
        with self.assertRaises(AuditError):close(1.,2.,'fixture',[])
    def test_constant_prediction_baseline(self):
        o=fixture();p=dict(encoder_prediction_rad=np.array([0.,0.,0.]),
            gyro_prediction_rad_s=np.array([0.,0.]),reported_current_prediction_A=np.array([.2,.2,.2]))
        m=compare(o,p,('encoder_q','gyro_v','decoded_current'))
        self.assertEqual(m['predicted_q_span_deg'],0.)
        self.assertAlmostEqual(m['q_rms_deg'],m['initial_observation_constant_q_rms_deg'])


if __name__=='__main__':unittest.main()
