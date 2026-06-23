import React from 'react';
import { Link } from 'react-router-dom';

const Lesson3 = () => {
  return (
    <div className="features-container" style={{ paddingTop: '2rem' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '2rem' }}>
        <h2 className="section-title" style={{ margin: 0 }}>Module 03: Vision-Language Models</h2>
        <div style={{ padding: '0.5rem 1rem', background: 'rgba(121,40,202,0.1)', color: 'var(--accent-purple)', borderRadius: '2rem', fontSize: '0.85rem', fontWeight: 600 }}>
          Advanced AI
        </div>
      </div>

      <div className="glass-panel" style={{ padding: '2rem', marginBottom: '3rem' }}>
        <p style={{ fontSize: '1.1rem', color: 'var(--text-secondary)', marginBottom: '2rem' }}>
          A Vision-Language Model (VLM) doesn't just "see" pixels—it connects visual understanding with structural language and reasoning. It provides total semantic grounding.
        </p>

        <div style={{ display: 'flex', gap: '2rem', flexWrap: 'wrap', flexDirection: 'row-reverse' }}>
          <div style={{ flex: '1 1 400px' }}>
            <img 
              src="/images/vlm_visual.png" 
              alt="VLM Cognitive Process"
              style={{ width: '100%', borderRadius: '12px', border: '1px solid var(--accent-purple)', boxShadow: '0 0 30px rgba(121, 40, 202, 0.2)' }}
            />
          </div>
          <div style={{ flex: '1 1 400px', display: 'flex', flexDirection: 'column', gap: '1.5rem', justifyContent: 'center' }}>
            <div className="glass-panel" style={{ padding: '1.5rem', background: 'rgba(0,0,0,0.4)', borderColor: 'rgba(255,255,255,0.05)' }}>
              <h4 className="text-gradient-alt" style={{ fontSize: '1.2rem', marginBottom: '0.5rem' }}>Traditional CV vs VLM</h4>
              <p style={{ color: 'var(--text-secondary)' }}>
                Traditional Computer Vision identifies bounded boxes ("Object: Cup, Confidence: 89%"). 
                <br /><br />
                A VLM understands context: "There is a blue cup placed precariously near the edge of the wooden table, which might fall if bumped."
              </p>
            </div>
            <div className="glass-panel" style={{ padding: '1.5rem', background: 'rgba(0,0,0,0.4)', borderColor: 'rgba(255,255,255,0.05)' }}>
              <h4 className="text-gradient-alt" style={{ fontSize: '1.2rem', marginBottom: '0.5rem' }}>Embodied AI Pipeline</h4>
              <p style={{ color: 'var(--text-secondary)' }}>
                For embodied robotics, a VLM sits between the sensor framework and the high-level policy planner, mapping raw camera topologies into structural language representations that can be logically executed.
              </p>
            </div>
          </div>
        </div>
      </div>

      <div style={{ display: 'flex', justifyContent: 'space-between', borderTop: '1px solid var(--border-color)', paddingTop: '2rem' }}>
        <Link to="/lesson-2" className="btn btn-secondary">← Previous: System Architecture</Link>
        <Link to="/lesson-4" className="btn btn-primary">Next: ROS 2 + VLM Integration →</Link>
      </div>
    </div>
  );
};

export default Lesson3;
