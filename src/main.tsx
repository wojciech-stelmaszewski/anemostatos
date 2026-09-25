import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import 'katex/dist/katex.min.css';
import './index.css';
import { App } from './App';
import { useParams } from './store/params';
import { sim, useUi } from './store/sim';

// Handy for poking at the simulation from the browser console.
if (import.meta.env.DEV) Object.assign(window, { sim, useParams, useUi });

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
