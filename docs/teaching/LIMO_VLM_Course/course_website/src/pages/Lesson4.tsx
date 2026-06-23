import React from 'react';
import { Link } from 'react-router-dom';

const Lesson4 = () => {
  return (
    <div className="features-container" style={{ paddingTop: '2rem' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '2rem' }}>
        <h2 className="section-title" style={{ margin: 0 }}>Module 04: ROS 2 + VLM Integration</h2>
        <div style={{ padding: '0.5rem 1rem', background: 'rgba(0,240,255,0.1)', color: 'var(--accent-cyan)', borderRadius: '2rem', fontSize: '0.85rem', fontWeight: 600 }}>
          System Integration
        </div>
      </div>

      <div className="glass-panel" style={{ padding: '2rem', marginBottom: '3rem' }}>
        <p style={{ fontSize: '1.1rem', color: 'var(--text-secondary)', marginBottom: '2rem' }}>
          This is where the magic happens. We tie the raw hardware feed from ROS 2 directly into the neural engine of the VLM to create an intelligent behavioral loop.
        </p>

        <div style={{ background: 'var(--bg-secondary)', borderRadius: '12px', padding: '2rem', display: 'flex', flexDirection: 'column', gap: '2rem' }}>
          <div style={{ padding: '1.5rem', borderLeft: '4px solid var(--accent-blue)', background: 'var(--bg-panel)' }}>
            <h3 style={{ marginBottom: '1rem', color: 'white' }}>1. The Semantic Pipeline</h3>
            <p style={{ color: 'var(--text-secondary)' }}>
              1. <code>robot_camera_node</code> publishes <code>sensor_msgs/Image</code> at 30Hz on <code>/camera/rgb/image_raw</code>. <br/>
              2. <code>vlm_inference_node</code> subscribes to that topic, converts the ROS image message into a tensor format suitable for the VLM (e.g. PyTorch Tensor). <br/>
              3. The VLM generates an analytical decision: <code>"Obstacle detected. Wait for 5 seconds."</code> <br/>
              4. A <code>vlm_parser_node</code> takes this textual command and emits structural geometric velocities mapping to <code>geometry_msgs/Twist</code> on <code>/cmd_vel</code>.
            </p>
          </div>
          
          <div style={{ display: 'flex', gap: '1rem', justifyContent: 'center', alignItems: 'center', color: 'var(--accent-cyan)' }}>
            <div style={{ padding: '1rem', background: 'rgba(0,240,255,0.05)', borderRadius: '8px', border: '1px solid rgba(0,240,255,0.2)' }}>Camera Topic</div>
            <div>→</div>
            <div style={{ padding: '1rem', background: 'rgba(121, 40, 202,0.1)', borderRadius: '8px', border: '1px solid rgba(121, 40, 202,0.3)', color: '#c084fc' }}>VLM Reasoning</div>
            <div>→</div>
            <div style={{ padding: '1rem', background: 'rgba(0,255,136,0.05)', borderRadius: '8px', border: '1px solid rgba(0,255,136,0.2)', color: '#4ade80' }}>Controller Command</div>
          </div>
        </div>
      </div>

      <div style={{ display: 'flex', justifyContent: 'space-between', borderTop: '1px solid var(--border-color)', paddingTop: '2rem' }}>
        <Link to="/lesson-3" className="btn btn-secondary">← Previous: Vision-Language Models</Link>
        <Link to="/lesson-5" className="btn btn-primary">Next: LIMO PRO Project Build →</Link>
      </div>
    </div>
  );
};

export default Lesson4;
