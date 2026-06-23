import React, { useState } from 'react';
import { Link } from 'react-router-dom';

const Lesson6 = () => {
  const [output, setOutput] = useState('');

  const runSimulation = (command: string) => {
    setOutput(`Processing: "${command}"\n[10.2312] Camera Node Published: /camera/image_raw (1920x1080)
[10.4132] VLM Node Subscribed: Processing visual inputs...
[11.5342] VLM Output: "The target object to ${command.toLowerCase()} has been localized. Trajectory clear."
[11.5361] Planner Node Published: /cmd_vel -> linear_x: 0.5, angular_z: 0.1
[12.1154] Motor Array: Executing command sequence...`);
  };

  return (
    <div className="features-container" style={{ paddingTop: '2rem' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '2rem' }}>
        <h2 className="section-title" style={{ margin: 0 }}>Module 06: Practical Scenarios</h2>
        <div style={{ padding: '0.5rem 1rem', background: 'rgba(255,255,255,0.1)', color: 'white', borderRadius: '2rem', fontSize: '0.85rem', fontWeight: 600 }}>
          Interactive Lab
        </div>
      </div>

      <div className="glass-panel" style={{ padding: '2rem', marginBottom: '3rem' }}>
        <p style={{ fontSize: '1.1rem', color: 'var(--text-secondary)', marginBottom: '2rem' }}>
          Simulate standard robot operations involving the full semantic toolchain. See how natural language effectively commands ROS 2 processes.
        </p>

        <div style={{ display: 'grid', gridTemplateColumns: '1fr 2fr', gap: '2rem' }}>
          <div>
            <h4 style={{ marginBottom: '1rem', color: 'var(--accent-cyan)' }}>Simulated User Input</h4>
            <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
              <button onClick={() => runSimulation("Inspect the Room")} className="btn btn-secondary" style={{ width: '100%', justifyContent: 'flex-start' }}>
                🎙️ "Inspect the Room"
              </button>
              <button onClick={() => runSimulation("Pick up the red bottle")} className="btn btn-secondary" style={{ width: '100%', justifyContent: 'flex-start' }}>
                🎙️ "Pick up the red bottle"
              </button>
              <button onClick={() => runSimulation("Move away from edge")} className="btn btn-secondary" style={{ width: '100%', justifyContent: 'flex-start' }}>
                🎙️ "Move away from edge"
              </button>
            </div>
          </div>

          <div style={{ background: '#000', borderRadius: '12px', border: '1px solid #333', display: 'flex', flexDirection: 'column', overflow: 'hidden' }}>
            <div style={{ padding: '0.75rem 1rem', background: '#111', borderBottom: '1px solid #333', fontSize: '0.85rem', color: '#888', fontFamily: 'monospace' }}>
              Terminal / ROS 2 System Log
            </div>
            <div style={{ padding: '1rem', flex: 1, fontFamily: 'monospace', color: '#00ff88', fontSize: '0.9rem', whiteSpace: 'pre-wrap', minHeight: '200px' }}>
              {output || 'Waiting for user interaction via semantic input interface...'}
            </div>
          </div>
        </div>
      </div>

      <div style={{ display: 'flex', justifyContent: 'space-between', borderTop: '1px solid var(--border-color)', paddingTop: '2rem' }}>
        <Link to="/lesson-5" className="btn btn-secondary">← Previous: LIMO PRO Build</Link>
        <Link to="/" className="btn btn-primary">Complete Course 🚀</Link>
      </div>
    </div>
  );
};

export default Lesson6;
