import React from 'react';
import { Link } from 'react-router-dom';

const Lesson2 = () => {
  return (
    <div className="features-container" style={{ paddingTop: '2rem' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '2rem' }}>
        <h2 className="section-title" style={{ margin: 0 }}>Module 02: Architecture & System Thinking</h2>
        <div style={{ padding: '0.5rem 1rem', background: 'rgba(0,112,243,0.1)', color: 'var(--accent-blue)', borderRadius: '2rem', fontSize: '0.85rem', fontWeight: 600 }}>
          Intermediate Level
        </div>
      </div>

      <div className="glass-panel" style={{ padding: '2rem', marginBottom: '3rem' }}>
        <p style={{ fontSize: '1.1rem', color: 'var(--text-secondary)', marginBottom: '2rem' }}>
          When building advanced robotics systems, you must move away from giant script files and start thinking in terms of <strong>distributed layers</strong>. Modularity is the key to scaling complex behaviors.
        </p>

        <div className="modules-list" style={{ marginTop: '3rem' }}>
          <div className="module-item" style={{ borderColor: 'var(--accent-cyan)' }}>
            <div className="module-number" style={{ opacity: 1, WebkitTextStroke: '1px var(--accent-cyan)', color: 'transparent' }}>Perception</div>
            <div className="module-content" style={{ paddingLeft: '2rem' }}>
              <h4 style={{ fontSize: '1.2rem', color: 'var(--text-primary)' }}>Information Gathering</h4>
              <p>Sensors like LiDARs, Depth Cameras, and Odometry feed raw data into the system. These nodes strictly handle formatting from hardware to ROS messages.</p>
            </div>
          </div>
          
          <div className="module-item" style={{ borderColor: 'var(--accent-purple)' }}>
            <div className="module-number" style={{ opacity: 1, WebkitTextStroke: '1px var(--accent-purple)', color: 'transparent' }}>Planning</div>
            <div className="module-content" style={{ paddingLeft: '2rem' }}>
              <h4 style={{ fontSize: '1.2rem', color: 'var(--text-primary)' }}>Decision Making</h4>
              <p>The reasoning core algorithms. This is where your AI model lives. It subscribes to perception topics, processes goals, and publishes trajectory or command velocity data.</p>
            </div>
          </div>

          <div className="module-item" style={{ borderColor: 'var(--accent-green)' }}>
            <div className="module-number" style={{ opacity: 1, WebkitTextStroke: '1px var(--accent-green)', color: 'transparent' }}>Control</div>
            <div className="module-content" style={{ paddingLeft: '2rem' }}>
              <h4 style={{ fontSize: '1.2rem', color: 'var(--text-primary)' }}>Execution</h4>
              <p>The lowest level. Controllers subscribe to the planner's output and output raw voltage or PWM signals to the LIMO PRO's wheel encoders and arm servos.</p>
            </div>
          </div>
        </div>
      </div>

      <div style={{ display: 'flex', justifyContent: 'space-between', borderTop: '1px solid var(--border-color)', paddingTop: '2rem' }}>
        <Link to="/lesson-1" className="btn btn-secondary">← Previous: ROS 2 Foundations</Link>
        <Link to="/lesson-3" className="btn btn-primary">Next: Vision-Language Models →</Link>
      </div>
    </div>
  );
};

export default Lesson2;
