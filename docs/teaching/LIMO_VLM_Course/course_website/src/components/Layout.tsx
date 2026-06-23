import React from 'react';
import { Outlet, Link, useLocation } from 'react-router-dom';

const Layout = () => {
  const location = useLocation();
  const isHome = location.pathname === '/';

  return (
    <div className="app-container">
      <nav className="main-nav">
        <Link to="/" className="nav-logo" style={{ textDecoration: 'none', color: 'inherit' }}>
          <div className="logo-dot"></div>
          LIMO PRO <span className="text-gradient">Academy</span>
        </Link>
        <div className="nav-links" style={{ display: 'flex', gap: '1rem', alignItems: 'center' }}>
          {!isHome && <Link to="/" className="nav-link">Home</Link>}
          <Link to="/lesson-1" className={`nav-link ${location.pathname.includes('lesson') ? 'active' : ''}`}>
            Course Modules
          </Link>
        </div>
      </nav>

      <main className="main-content">
        <Outlet />
      </main>

      <footer style={{ marginTop: 'auto', padding: '2rem', textAlign: 'center', borderTop: '1px solid var(--border-color)', background: 'var(--bg-secondary)' }}>
        <p style={{ color: 'var(--text-secondary)', fontSize: '0.9rem' }}>
          LIMO PRO + VLM Advanced Robotics Course • Internal Dashboard
        </p>
      </footer>
    </div>
  );
};

export default Layout;
