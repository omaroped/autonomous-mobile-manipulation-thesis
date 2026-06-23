import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import Layout from './components/Layout';
import Home from './pages/Home';
import Lesson1 from './pages/Lesson1';
import Lesson2 from './pages/Lesson2';
import Lesson3 from './pages/Lesson3';
import Lesson4 from './pages/Lesson4';
import Lesson5 from './pages/Lesson5';
import Lesson6 from './pages/Lesson6';
import './index.css';

function App() {
  return (
    <Router>
      <Routes>
        <Route path="/" element={<Layout />}>
          <Route index element={<Home />} />
          <Route path="lesson-1" element={<Lesson1 />} />
          <Route path="lesson-2" element={<Lesson2 />} />
          <Route path="lesson-3" element={<Lesson3 />} />
          <Route path="lesson-4" element={<Lesson4 />} />
          <Route path="lesson-5" element={<Lesson5 />} />
          <Route path="lesson-6" element={<Lesson6 />} />
        </Route>
      </Routes>
    </Router>
  );
}

export default App;
