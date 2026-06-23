import React from 'react';
import { Link } from 'react-router-dom';

const Home = () => {
  return (
    <>
      <section className="hero-section">
        <div className="hero-badge">Course Master Plan • Interactive Learning</div>
        <h1 className="hero-title">
          Build Intelligent Embodied Systems with <span className="text-gradient">ROS 2 & VLMs</span>
        </h1>
        <p className="hero-subtitle">
          A comprehensive, visual, and highly technical journey from ROS 2 foundations to deploying Vision-Language Models on the LIMO PRO mobile manipulator.
        </p>
        <div className="hero-cta">
          <Link to="/lesson-1">
            <button className="btn btn-primary">Start Learning</button>
          </Link>
          <a href="#roadmap">
            <button className="btn btn-secondary">Explore Architecture</button>
          </a>
        </div>
      </section>

      <section id="roadmap" className="features-container">
        <span className="section-tag">Curriculum Overview</span>
        <h2 className="section-title">Course Modules</h2>
        
        <div className="feature-grid">
          <Link to="/lesson-1" className="feature-card glass-panel" style={{ textDecoration: 'none', color: 'inherit' }}>
            <div className="feature-icon">🤖</div>
            <h3>01. ROS 2 Foundations</h3>
            <p>Master the basics: Nodes, Topics, Services, Actions, Parameters, and DDS middleware configuration.</p>
          </Link>
          
          <Link to="/lesson-2" className="feature-card glass-panel" style={{ textDecoration: 'none', color: 'inherit' }}>
            <div className="feature-icon">⚙️</div>
            <h3>02. Architecture & Thinking</h3>
            <p>Move from isolated functions to a distributed system. Learn perception, planning, and control layer decoupling.</p>
          </Link>
          
          <Link to="/lesson-3" className="feature-card glass-panel" style={{ textDecoration: 'none', color: 'inherit' }}>
            <div className="feature-icon">👁️</div>
            <h3>03. Vision-Language Models</h3>
            <p>Understand semantic reasoning and scene interpretation. Move beyond traditional computer vision entirely.</p>
          </Link>

          <Link to="/lesson-4" className="feature-card glass-panel" style={{ textDecoration: 'none', color: 'inherit' }}>
            <div className="feature-icon">🔗</div>
            <h3>04. ROS 2 + VLM Integration</h3>
            <p>Bridging the middleware with the AI inference node. Connecting the camera topic to language processing.</p>
          </Link>

          <Link to="/lesson-5" className="feature-card glass-panel" style={{ textDecoration: 'none', color: 'inherit' }}>
            <div className="feature-icon">🏎️</div>
            <h3>05. LIMO PRO Project Build</h3>
            <p>Complete architecture breakdown of the 4-wheel base, robotic arm, and perception stack in action.</p>
          </Link>

          <Link to="/lesson-6" className="feature-card glass-panel" style={{ textDecoration: 'none', color: 'inherit' }}>
            <div className="feature-icon">🎯</div>
            <h3>06. Practical Scenarios</h3>
            <p>Run end-to-end tests: detecting objects, driving toward them safely, and verbally describing the environment.</p>
          </Link>
        </div>
      </section>
    </>
  );
};

export default Home;
