#!/usr/bin/env python3
"""
Simple test client for the SLAM server
Demonstrates how to send images and receive pose estimates
"""

import requests
import json
import base64
import numpy as np
import cv2
import time
import argparse

class SLAMClient:
    def __init__(self, server_url="http://localhost:8080"):
        self.server_url = server_url
        self.session = requests.Session()
        
    def health_check(self):
        """Check if the server is healthy"""
        try:
            response = self.session.get(f"{self.server_url}/api/v1/health", timeout=5)
            if response.status_code == 200:
                data = response.json()
                print(f"Server is healthy: {data}")
                return True
            else:
                print(f"Health check failed: {response.status_code}")
                return False
        except Exception as e:
            print(f"Health check error: {e}")
            return False
    
    def get_status(self):
        """Get server status"""
        try:
            response = self.session.get(f"{self.server_url}/api/v1/status", timeout=5)
            if response.status_code == 200:
                return response.json()
            else:
                print(f"Status request failed: {response.status_code}")
                return None
        except Exception as e:
            print(f"Status request error: {e}")
            return None
    
    def track_image(self, image, timestamp=None, frame_id=None, intrinsics=None):
        """Send image for tracking"""
        if timestamp is None:
            timestamp = time.time()
        
        if frame_id is None:
            frame_id = int(timestamp * 1000)
        
        # Encode image as base64
        _, buffer = cv2.imencode('.png', image)
        image_base64 = base64.b64encode(buffer).decode('utf-8')
        
        # Prepare request
        data = {
            "image_base64": image_base64,
            "timestamp": timestamp,
            "frame_id": frame_id
        }
        
        if intrinsics is not None:
            data["intrinsics"] = intrinsics
        
        try:
            response = self.session.post(
                f"{self.server_url}/api/v1/track",
                json=data,
                headers={"Content-Type": "application/json", "X-Client-ID": "test_client"},
                timeout=10
            )
            
            if response.status_code == 200:
                return response.json()
            else:
                print(f"Track request failed: {response.status_code} - {response.text}")
                return None
        except Exception as e:
            print(f"Track request error: {e}")
            return None
    
    def get_trajectory(self):
        """Get current trajectory"""
        try:
            response = self.session.get(f"{self.server_url}/api/v1/trajectory", timeout=5)
            if response.status_code == 200:
                return response.json()
            else:
                print(f"Trajectory request failed: {response.status_code}")
                return None
        except Exception as e:
            print(f"Trajectory request error: {e}")
            return None

def create_test_image(width=640, height=480, pattern="grid"):
    """Create a test image with features"""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    
    if pattern == "grid":
        # Create a grid pattern
        for i in range(0, height, 20):
            cv2.line(img, (0, i), (width, i), (100, 100, 100), 1)
        for j in range(0, width, 20):
            cv2.line(img, (j, 0), (j, height), (100, 100, 100), 1)
        
        # Add some circles
        for i in range(5):
            for j in range(8):
                center = (j * 80 + 40, i * 80 + 40)
                cv2.circle(img, center, 10, (255, 255, 255), -1)
    
    elif pattern == "checkerboard":
        # Create checkerboard pattern
        square_size = 40
        for i in range(0, height, square_size):
            for j in range(0, width, square_size):
                if (i // square_size + j // square_size) % 2 == 0:
                    cv2.rectangle(img, (j, i), (j + square_size, i + square_size), (255, 255, 255), -1)
    
    return img

def main():
    parser = argparse.ArgumentParser(description="Test client for SLAM server")
    parser.add_argument("--server", default="http://localhost:8080", help="Server URL")
    parser.add_argument("--frames", type=int, default=10, help="Number of test frames to send")
    parser.add_argument("--pattern", choices=["grid", "checkerboard"], default="grid", help="Test image pattern")
    args = parser.parse_args()
    
    client = SLAMClient(args.server)
    
    print("Testing SLAM Server...")
    print(f"Server URL: {args.server}")
    
    # Health check
    print("\n1. Health Check:")
    if not client.health_check():
        print("Server is not responding, exiting...")
        return
    
    # Get initial status
    print("\n2. Initial Status:")
    status = client.get_status()
    if status:
        print(json.dumps(status, indent=2))
    
    # Send test images
    print(f"\n3. Sending {args.frames} test images:")
    for i in range(args.frames):
        # Create test image with slight variations
        img = create_test_image(pattern=args.pattern)
        
        # Add some noise to make each frame slightly different
        noise = np.random.randint(-10, 10, img.shape, dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        # Optional: Add camera intrinsics (example values)
        intrinsics = [500.0, 500.0, 320.0, 240.0]  # fx, fy, cx, cy
        
        print(f"  Sending frame {i+1}...")
        result = client.track_image(img, intrinsics=intrinsics)
        if result:
            print(f"    Result: {result}")
        
        time.sleep(0.1)  # Small delay between frames
    
    # Get final status
    print("\n4. Final Status:")
    status = client.get_status()
    if status:
        print(json.dumps(status, indent=2))
    
    # Get trajectory
    print("\n5. Trajectory:")
    trajectory = client.get_trajectory()
    if trajectory:
        print(f"Number of poses: {trajectory.get('count', 0)}")
        if trajectory.get('trajectory'):
            print("Recent poses:")
            for pose in trajectory['trajectory'][-3:]:  # Show last 3 poses
                print(f"  Frame {pose['frame_id']}: pos={pose['position']}, "
                      f"time={pose['processing_time_ms']:.1f}ms")

if __name__ == "__main__":
    main()