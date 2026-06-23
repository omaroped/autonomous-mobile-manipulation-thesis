import { useState } from 'react';
import { Link } from 'react-router-dom';
import { motion, AnimatePresence } from 'framer-motion';
import { Network, Zap, Package, Radio, Settings, ChevronRight, BookOpen, Terminal, CheckCircle2 } from 'lucide-react';

// Reusable icons for SVG
const Camera = ({size, className}: any) => <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" className={className}><path d="M14.5 4h-5L7 7H4a2 2 0 0 0-2 2v9a2 2 0 0 0 2 2h16a2 2 0 0 0 2-2V9a2 2 0 0 0-2-2h-3l-2.5-3z"/><circle cx="12" cy="13" r="3"/></svg>
const Cpu = ({size, className}: any) => <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" className={className}><rect x="4" y="4" width="16" height="16" rx="2" ry="2"/><rect x="9" y="9" width="6" height="6"/><line x1="9" y1="1" x2="9" y2="4"/><line x1="15" y1="1" x2="15" y2="4"/><line x1="9" y1="20" x2="9" y2="23"/><line x1="15" y1="20" x2="15" y2="23"/><line x1="20" y1="9" x2="23" y2="9"/><line x1="20" y1="14" x2="23" y2="14"/><line x1="1" y1="9" x2="4" y2="9"/><line x1="1" y1="14" x2="4" y2="14"/></svg>


const steps = [
  {
    id: 'intro',
    title: 'Introduction to ROS 2',
    icon: <BookOpen size={24} />,
    content: (
      <div>
        <p className="mb-4">ROS 2 (Robot Operating System 2) is the industry standard middleware for robotics. Unlike traditional operating systems (Windows, Linux), ROS 2 runs <em>on top</em> of a host OS and provides a structured way for different parts of a robot (sensors, motors, AI models) to talk to each other.</p>
        <div className="bg-blue-900/20 border border-blue-500/30 p-4 rounded-lg">
          <h4 className="flex items-center gap-2 text-blue-400 font-semibold mb-2"><CheckCircle2 size={18} /> Core Philosophy</h4>
          <p className="text-sm text-blue-200">Deconstruct complex monolithic programs into many small, independent programs (Nodes) that communicate over a standardized network graph.</p>
        </div>
      </div>
    )
  },
  {
    id: 'nodes',
    title: 'Nodes: The Executable Units',
    icon: <Package size={24} />,
    content: (
      <div>
        <p className="mb-4">A <strong>Node</strong> is a single, self-contained process that performs a specific task. By keeping tasks separated, a crash in the camera node won't take down the motor controls.</p>
        
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4 my-6">
          <div className="p-4 rounded-xl border border-[rgba(255,255,255,0.05)] bg-[rgba(0,0,0,0.3)]">
            <h5 className="text-green-400 font-mono text-sm mb-2">sensor_node.py</h5>
            <p className="text-sm text-gray-400">Reads raw bytes from the LIMO PRO's depth camera and formats them into an image.</p>
          </div>
          <div className="p-4 rounded-xl border border-[rgba(255,255,255,0.05)] bg-[rgba(0,0,0,0.3)]">
            <h5 className="text-blue-400 font-mono text-sm mb-2">vlm_inference.cpp</h5>
            <p className="text-sm text-gray-400">Takes the formatted image, runs neural net inference, and outputs semantic text.</p>
          </div>
        </div>
      </div>
    )
  },
  {
    id: 'topics',
    title: 'Topics & Publish/Subscribe',
    icon: <Radio size={24} />,
    content: (
      <div>
        <p className="mb-4">Nodes communicate asynchronously using <strong>Topics</strong>. This follows a Publisher-Subscriber pattern. A node can publish data to a topic, and any number of other nodes can subscribe to that topic to receive the data stream.</p>
        
        <div className="relative p-6 rounded-xl border border-[rgba(0,240,255,0.2)] bg-[rgba(0,240,255,0.05)] flex flex-col md:flex-row items-center justify-between gap-4">
           <div className="text-center">
             <div className="w-16 h-16 rounded-full bg-blue-900/50 flex flex-col items-center justify-center border border-blue-500 mx-auto mb-2">
                 <Camera className="text-blue-400" size={24}/>
             </div>
             <span className="text-xs font-mono">Camera Node</span><br/>
             <span className="text-[10px] text-gray-500">(Publisher)</span>
           </div>
           
           <div className="flex-1 flex flex-col items-center text-[#00f0ff] font-mono text-sm font-bold relative w-full">
              <span className="bg-[#0a0a0f] px-4 py-1 rounded-full border border-[rgba(0,240,255,0.3)] z-10 block mb-2">/camera/image_raw</span>
              <div className="h-[2px] w-full bg-gradient-to-r from-blue-500 to-purple-500 absolute top-1/2 -translate-y-1/2 z-0 opacity-50"></div>
              <span className="text-xs font-normal text-gray-400 mt-1">sensor_msgs/Image</span>
           </div>

           <div className="text-center">
             <div className="w-16 h-16 rounded-full bg-purple-900/50 flex flex-col items-center justify-center border border-purple-500 mx-auto mb-2">
                 <Cpu className="text-purple-400" size={24}/>
             </div>
             <span className="text-xs font-mono">VLM Node</span><br/>
             <span className="text-[10px] text-gray-500">(Subscriber)</span>
           </div>
        </div>
      </div>
    )
  },
  {
    id: 'services',
    title: 'Services & Actions',
    icon: <Zap size={24} />,
    content: (
      <div>
         <p className="mb-4">While topics are for continuous streams (like video), sometimes you need different interaction patterns:</p>
         
         <div className="space-y-4">
           <div className="p-4 border-l-4 border-yellow-500 bg-yellow-500/10 rounded-r-lg">
             <h4 className="font-bold text-yellow-500 mb-1">Services (Synchronous)</h4>
             <p className="text-sm text-gray-300">Request / Response. Used for quick queries. <em>Ex: "What is the battery level?" {"->"} "87%"</em>.</p>
           </div>
           
           <div className="p-4 border-l-4 border-emerald-500 bg-emerald-500/10 rounded-r-lg">
             <h4 className="font-bold text-emerald-500 mb-1">Actions (Asynchronous with Feedback)</h4>
             <p className="text-sm text-gray-300">Used for long-running tasks where you want progress updates and the ability to cancel. <em>Ex: "Navigate to the kitchen." {"->"} (Updates: 20% there... 50% there...) {"->"} "Arrived."</em></p>
           </div>
         </div>
      </div>
    )
  },
  {
    id: 'params',
    title: 'Parameters & DDS',
    icon: <Settings size={24} />,
    content: (
      <div>
        <h4 className="font-bold text-white mb-2">Parameters</h4>
        <p className="mb-4 text-gray-300">Configuration values (integers, floats, booleans) that can be changed at runtime without recompiling the code. E.g., changing the <code>max_speed</code> of the robot on the fly.</p>
        
        <h4 className="font-bold text-white mb-2 mt-6">DDS (Data Distribution Service)</h4>
        <p className="text-gray-300">The secret sauce of ROS 2. It is the underlying networking middleware that handles node discovery, message serialization, and ensures delivery across different physical computers (e.g., your laptop talking seamlessly to the Raspberry Pi on the LIMO PRO) without needing a central "Master" node.</p>
      </div>
    )
  },
  {
    id: 'interactive',
    title: 'System Architecture Visualization',
    icon: <Network size={24} />,
    content: (
      <div className="flex flex-col items-center">
        <p className="text-center text-gray-400 mb-4 max-w-lg">
          The result of these foundations is a fully decentralized, highly modular "ROS Graph" where intelligence and hardware abstractly exchange data.
        </p>
        <img 
          src="/images/ros2_dashboard.png" 
          alt="ROS 2 Architecture Visualization"
          className="w-full max-w-2xl rounded-xl border border-[rgba(255,255,255,0.1)] shadow-2xl shadow-blue-900/20"
        />
      </div>
    )
  }
];

const Lesson1 = () => {
  const [currentStep, setCurrentStep] = useState(0);

  const nextStep = () => {
    if (currentStep < steps.length - 1) setCurrentStep(curr => curr + 1);
  };

  const prevStep = () => {
    if (currentStep > 0) setCurrentStep(curr => curr - 1);
  };

  return (
    <div className="max-w-5xl mx-auto pt-8 px-4">
      {/* Header */}
      <div className="flex flex-col md:flex-row justify-between md:items-end mb-8 gap-4 border-b border-[rgba(255,255,255,0.1)] pb-6">
        <div>
          <span className="text-[var(--accent-cyan)] font-mono text-sm tracking-wider uppercase">Module 01</span>
          <h1 className="text-4xl md:text-5xl font-bold mt-2 font-['Outfit']">ROS 2 Foundations</h1>
        </div>
        <div className="bg-[rgba(0,255,136,0.1)] text-[var(--accent-green)] px-4 py-2 rounded-full text-sm font-semibold inline-flex items-center gap-2">
          <Terminal size={16} /> Technical Deep Dive
        </div>
      </div>

      {/* Main Interactive Container */}
      <div className="flex flex-col lg:flex-row gap-8 min-h-[500px]">
        
        {/* Sidebar Navigation */}
        <div className="lg:w-1/3 flex flex-col gap-2">
          {steps.map((step, index) => {
            const isActive = currentStep === index;
            const isPast = currentStep > index;
            return (
              <button
                key={step.id}
                onClick={() => setCurrentStep(index)}
                className={`flex items-center gap-4 p-4 rounded-xl text-left transition-all duration-300 border
                  ${isActive 
                    ? 'bg-[var(--bg-panel)] border-[var(--glass-border)] shadow-[0_0_20px_rgba(0,240,255,0.1)] text-white' 
                    : isPast
                      ? 'bg-transparent border-transparent text-gray-400 hover:bg-[rgba(255,255,255,0.02)]' 
                      : 'bg-transparent border-transparent text-gray-600 hover:text-gray-400'
                  }`}
              >
                <div className={`p-2 rounded-lg ${isActive ? 'bg-gradient-to-br from-[var(--accent-cyan)] to-[var(--accent-blue)] text-white shadow-lg' : 'bg-[#1a1a24]'}`}>
                  {step.icon}
                </div>
                <div>
                  <div className="text-xs font-mono mb-1 opacity-70">Step 0{index + 1}</div>
                  <div className="font-semibold">{step.title}</div>
                </div>
              </button>
            )
          })}
        </div>

        {/* Content Area */}
        <div className="lg:w-2/3 glass-panel relative overflow-hidden flex flex-col">
          {/* Progress bar */}
          <div className="h-1 w-full bg-[#1a1a24] absolute top-0 left-0">
            <motion.div 
              className="h-full bg-gradient-to-r from-[var(--accent-cyan)] to-[var(--accent-purple)]"
              initial={{ width: 0 }}
              animate={{ width: `${((currentStep + 1) / steps.length) * 100}%` }}
              transition={{ duration: 0.3 }}
            />
          </div>

          <div className="p-8 flex-1">
            <AnimatePresence mode="wait">
              <motion.div
                key={currentStep}
                initial={{ opacity: 0, y: 10, filter: 'blur(5px)' }}
                animate={{ opacity: 1, y: 0, filter: 'blur(0px)' }}
                exit={{ opacity: 0, y: -10, filter: 'blur(5px)' }}
                transition={{ duration: 0.3 }}
              >
                <h3 className="text-2xl font-bold mb-6 flex items-center gap-3">
                  {steps[currentStep].title}
                </h3>
                <div className="text-[var(--text-primary)] leading-relaxed text-lg preserve-tailwind" style={{ }}>
                  {steps[currentStep].content}
                </div>
              </motion.div>
            </AnimatePresence>
          </div>

          {/* Bottom Controls */}
          <div className="p-6 border-t border-[rgba(255,255,255,0.05)] bg-[rgba(0,0,0,0.2)] flex justify-between items-center mt-auto">
            <button 
              onClick={prevStep}
              disabled={currentStep === 0}
              className={`px-4 py-2 rounded-lg font-semibold transition-colors
                ${currentStep === 0 ? 'text-gray-600 cursor-not-allowed' : 'text-gray-300 hover:text-white bg-[rgba(255,255,255,0.05)] hover:bg-[rgba(255,255,255,0.1)]'}`}
            >
              Previous
            </button>
            <button 
              onClick={nextStep}
              disabled={currentStep === steps.length - 1}
              className={`px-6 py-2 rounded-lg font-semibold flex items-center gap-2 transition-all
                ${currentStep === steps.length - 1 
                  ? 'opacity-0 pointer-events-none' 
                  : 'bg-[var(--accent-blue)] text-white hover:shadow-[0_0_15px_rgba(0,112,243,0.4)] hover:scale-105'}`}
            >
              Next Step <ChevronRight size={18} />
            </button>
          </div>
        </div>
      </div>

      <div className="flex justify-between border-t border-[var(--border-color)] pt-8 mt-12 mb-12">
        <Link to="/" className="text-gray-400 hover:text-white transition-colors">← Back to Overview</Link>
        <Link to="/lesson-2" className="text-[var(--accent-cyan)] font-semibold hover:text-white flex items-center gap-2 transition-colors">
          Proceed to Module 02 <ChevronRight size={18} />
        </Link>
      </div>
    </div>
  );
};

export default Lesson1;
