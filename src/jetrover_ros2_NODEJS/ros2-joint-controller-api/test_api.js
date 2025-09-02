#!/usr/bin/env node

const axios = require('axios');

const API_BASE = 'http://localhost:3000';

// Add axios as dependency if not already installed
// npm install axios

async function testAPI() {
    console.log('🧪 Testing ROS2 Joint Controller API\n');
    
    try {
        // Test 1: Health check
        console.log('1️⃣  Testing health endpoint...');
        const healthResponse = await axios.get(`${API_BASE}/api/health`);
        console.log('✅ Health check:', healthResponse.data);
        console.log();

        // Test 2: Get available joints
        console.log('2️⃣  Getting available joints...');
        const jointsResponse = await axios.get(`${API_BASE}/api/joints`);
        console.log('✅ Available joints:', jointsResponse.data);
        console.log();

        // Test 3: Move single joint
        console.log('3️⃣  Moving single joint (joint2 to 1.5708 rad = 90°)...');
        const singleJointResponse = await axios.post(`${API_BASE}/api/joint/joint2/move`, {
            position: 1.5708
        });
        console.log('✅ Single joint move:', singleJointResponse.data);
        console.log();

        // Wait a bit
        await sleep(2000);

        // Test 4: Move multiple joints
        console.log('4️⃣  Moving multiple joints...');
        const multiJointResponse = await axios.post(`${API_BASE}/api/joints/move`, {
            joints: {
                joint1: 0.7854,  // 45°
                joint2: -0.7854, // -45°
                joint3: 1.5708   // 90°
            }
        });
        console.log('✅ Multiple joint move:', multiJointResponse.data);
        console.log();

        // Wait a bit
        await sleep(2000);

        // Test 5: Get available poses
        console.log('5️⃣  Getting available poses...');
        const posesResponse = await axios.get(`${API_BASE}/api/poses`);
        console.log('✅ Available poses:', posesResponse.data);
        console.log();

        // Test 6: Execute predefined pose
        console.log('6️⃣  Executing home pose...');
        const poseResponse = await axios.post(`${API_BASE}/api/pose/home`);
        console.log('✅ Pose execution:', poseResponse.data);
        console.log();

        // Test 7: Animation sequence
        console.log('7️⃣  Running animation sequence...');
        await animationSequence();

        console.log('🎉 All tests completed successfully!');

    } catch (error) {
        console.error('❌ Test failed:', error.response?.data || error.message);
        
        if (error.code === 'ECONNREFUSED') {
            console.log('\n💡 Make sure the API server is running:');
            console.log('   npm start');
        }
    }
}

async function animationSequence() {
    const sequences = [
        { joint1: 0, joint2: 0, joint3: 0 },           // Home
        { joint1: 1.57, joint2: 0.78, joint3: -0.78 }, // Pose 1
        { joint1: -1.57, joint2: 1.57, joint3: 0 },    // Pose 2
        { joint1: 0, joint2: -1.57, joint3: 1.57 },    // Custom pose
        { joint1: 0, joint2: 0, joint3: 0 }            // Back to home
    ];

    for (let i = 0; i < sequences.length; i++) {
        console.log(`   Step ${i + 1}/${sequences.length}:`, sequences[i]);
        
        try {
            await axios.post(`${API_BASE}/api/joints/move`, {
                joints: sequences[i]
            });
            await sleep(1500); // Wait 1.5 seconds between moves
        } catch (error) {
            console.error(`   ❌ Animation step ${i + 1} failed:`, error.message);
        }
    }
    
    console.log('✅ Animation sequence completed');
}

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// Add some additional utility functions
async function testErrorHandling() {
    console.log('🔍 Testing error handling...\n');
    
    // Test invalid joint name
    try {
        await axios.post(`${API_BASE}/api/joint/invalid_joint/move`, {
            position: 1.0
        });
    } catch (error) {
        console.log('✅ Invalid joint name handled correctly:', error.response.data);
    }
    
    // Test invalid position
    try {
        await axios.post(`${API_BASE}/api/joint/joint1/move`, {
            position: "not_a_number"
        });
    } catch (error) {
        console.log('✅ Invalid position handled correctly:', error.response.data);
    }
    
    // Test invalid pose
    try {
        await axios.post(`${API_BASE}/api/pose/invalid_pose`);
    } catch (error) {
        console.log('✅ Invalid pose handled correctly:', error.response.data);
    }
    
    console.log();
}

async function continuousTest() {
    console.log('🔄 Running continuous joint movement test...\n');
    
    const jointName = 'joint2';
    const amplitude = 1.5; // radians
    const frequency = 0.5; // Hz
    const duration = 10000; // 10 seconds
    
    const startTime = Date.now();
    
    while (Date.now() - startTime < duration) {
        const t = (Date.now() - startTime) / 1000; // time in seconds
        const position = amplitude * Math.sin(2 * Math.PI * frequency * t);
        
        try {
            await axios.post(`${API_BASE}/api/joint/${jointName}/move`, { position });
            console.log(`t=${t.toFixed(1)}s: ${jointName} = ${position.toFixed(3)} rad`);
        } catch (error) {
            console.error('Movement failed:', error.message);
        }
        
        await sleep(100); // 10 Hz update rate
    }
    
    // Return to home
    await axios.post(`${API_BASE}/api/joint/${jointName}/move`, { position: 0 });
    console.log('✅ Continuous test completed, returned to home position\n');
}

// Command line interface
if (require.main === module) {
    const args = process.argv.slice(2);
    
    if (args.includes('--continuous')) {
        continuousTest();
    } else if (args.includes('--errors')) {
        testErrorHandling();
    } else if (args.includes('--help')) {
        console.log(`
Usage: node test_api.js [options]

Options:
  --continuous    Run continuous sine wave movement test
  --errors        Test error handling scenarios  
  --help          Show this help message

Examples:
  node test_api.js                    # Run basic API tests
  node test_api.js --continuous       # Run continuous movement test
  node test_api.js --errors           # Test error scenarios
        `);
    } else {
        testAPI();
    }
}