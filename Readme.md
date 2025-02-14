# Code Analysis Test Project

This project provides a complex codebase for testing and validating code analysis tools, particularly those focused on:
- Attack surface extraction
- Code structure analysis
- Inter-process communication patterns
- Component dependency mapping
- Security boundary identification

## Project Structure

The project simulates an airplane control system purely as a way to generate a realistic, complex codebase with multiple interacting components. While it uses aviation terminology, it is **not intended for any actual aviation or aircraft-related use**.

Key architectural elements for testing analysis tools:

### Core System Components
- Message bus system using shared memory (IPC)
- Multiple forked processes
- Component status monitoring
- State management system

### Network Components
- TCP socket communications
- Multiple network services (GPS, Radio, Satellite)
- Client-server architecture
- Connection management

### Component Organization
```
project/
├── include/          # Header files defining interfaces
├── src/
│   ├── core/         # Core system components
│   ├── components/   # Individual system components
│   └── external/     # Network service simulators
├── config/           # Configuration files
└── build/           # Build output
```

## Features for Testing Tools

### IPC Mechanisms
- Shared memory segments
- Message passing system
- Semaphores for synchronization
- Socket communications

### Security Boundaries
- Process isolation
- Network service boundaries
- Data validation points
- Error handling paths

### Component Interactions
- Publish/subscribe messaging
- State updates
- System status monitoring
- Error propagation

## Building and Running

### Dependencies
```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install build-essential libjson-c-dev
```

### Build
```bash
make clean
make
```

### Run
```bash
./start_simulation.sh
```

### Clean Up
```bash
./cleanup.sh
```

## Usage for Analysis Tools

This project is specifically designed to test tools that:
1. Extract attack surfaces from complex codebases
2. Map component interactions and dependencies
3. Identify security boundaries and trust zones
4. Analyze error propagation paths
5. Validate data flow patterns

### Example Analysis Targets

1. Message Bus System
   - Shared memory access patterns
   - Message validation points
   - Component permissions

2. Network Services
   - Socket initialization
   - Data parsing
   - Error handling
   - Connection management

3. Component Architecture
   - Process isolation
   - State management
   - Error propagation
   - Resource cleanup

## Note

This project is NOT:
- An actual aircraft simulation
- Suitable for aviation-related use
- A reference for aircraft systems

It is ONLY intended as a test bed for code analysis tools.

## License

BSD 3-Clause License - See LICENSE file for details
