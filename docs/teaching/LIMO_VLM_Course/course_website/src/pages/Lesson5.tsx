import React from 'react';
import { Link } from 'react-router-dom';

const Lesson5 = () => {
  return (
    <div className="features-container" style={{ paddingTop: '2rem' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '2rem' }}>
        <h2 className="section-title" style={{ margin: 0 }}>Module 05: LIMO PRO Build</h2>
        <div style={{ padding: '0.5rem 1rem', background: 'rgba(0,255,136,0.1)', color: 'var(--accent-green)', borderRadius: '2rem', fontSize: '0.85rem', fontWeight: 600 }}>
          Hardware Case Study
        </div>
      </div>

      <div className="glass-panel" style={{ padding: '2rem', marginBottom: '3rem' }}>
        <p style={{ fontSize: '1.1rem', color: 'var(--text-secondary)', marginBottom: '2rem' }}>
          Tying the theory directly to hardware. The LIMO PRO features omnidirectional or Ackermann steering, an integrated arm, and an Orbbec Astra depth camera framework.
        </p>

        <div style={{ display: 'flex', gap: '3rem', flexDirection: 'column' }}>
          <img 
            src="/images/limo_pro.png" 
            alt="LIMO PRO Cinematic Digital Twin"
            style={{ width: '100%', maxHeight: '450px', objectFit: 'cover', borderRadius: '12px', border: '1px solid var(--accent-green)', boxShadow: '0 0 40px rgba(0,255,136,0.1)' }}
          />
          
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))', gap: '1.5rem' }}>
            <div style={{ padding: '1.5rem', background: 'var(--bg-panel)', borderRadius: '12px', border: '1px solid var(--border-color)' }}>
              <h4 style={{ color: 'var(--accent-cyan)', marginBottom: '0.5rem' }}>Kinematics</h4>
              <p style={{ fontSize: '0.9rem', color: 'var(--text-secondary)' }}>The base supports 4-wheel independent driving. This gives you extreme versatility in how the VLM dictates motion patterns.</p>
            </div>
            <div style={{ padding: '1.5rem', background: 'var(--bg-panel)', borderRadius: '12px', border: '1px solid var(--border-color)' }}>
              <h4 style={{ color: 'var(--accent-cyan)', marginBottom: '0.5rem' }}>Camera Payload</h4>
              <p style={{ fontSize: '0.9rem', color: 'var(--text-secondary)' }}>Standard Astra feed provides both RGB images for the VLM inference and Depth PointClouds for critical collision-avoidance fallbacks.</p>
            </div>
            <div style={{ padding: '1.5rem', background: 'var(--bg-panel)', borderRadius: '12px', border: '1px solid var(--border-color)' }}>
              <h4 style={{ color: 'var(--accent-cyan)', marginBottom: '0.5rem' }}>Manipulation Arm</h4>
              <p style={{ fontSize: '0.9rem', color: 'var(--text-secondary)' }}>A multi-DOF arm that allows the robot to execute 'pick' actions governed by semantic goals identified by the VLM object detection layer.</p>
            </div>
          </div>
        </div>
      </div>

      <div style={{ display: 'flex', justifyContent: 'space-between', borderTop: '1px solid var(--border-color)', paddingTop: '2rem' }}>
        <Link to="/lesson-4" className="btn btn-secondary">← Previous: ROS 2 + VLM Integration</Link>
        <Link to="/lesson-6" className="btn btn-primary">Next: Practical Scenarios →</Link>
      </div>
    </div>
  );
};

export default Lesson5;
