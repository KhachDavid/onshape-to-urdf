#!/usr/bin/env python3
"""
Onshape to URDF Conversion Pipeline
Main script for converting Onshape assemblies to URDF robot descriptions.
"""

import os
import sys
import json
import argparse
import subprocess
from pathlib import Path
from datetime import datetime
import xml.etree.ElementTree as ET
from typing import Dict


class OnshapeToURDFConverter:
    """Main converter class for Onshape to URDF pipeline."""
    
    def __init__(self, config_path, output_dir=None, verbose=False, standalone=False):
        """
        Initialize the converter.
        
        Args:
            config_path: Path to config.json file
            output_dir: Output directory for URDF files (optional)
            verbose: Enable verbose output
        """
        self.config_path = Path(config_path)
        self.verbose = verbose
        self.output_dir = Path(output_dir) if output_dir else Path("output")
        self.standalone = standalone
        
        # Load configuration
        self.config = self._load_config()
        
        # Validate environment
        self._validate_environment()
    
    def _load_config(self):
        """Load and validate configuration file."""
        if not self.config_path.exists():
            raise FileNotFoundError(f"Configuration file not found: {self.config_path}")
        
        with open(self.config_path, 'r') as f:
            config = json.load(f)
        
        # Validate required fields
        if 'documentId' not in config or config['documentId'] == 'YOUR_DOCUMENT_ID':
            raise ValueError("Please set 'documentId' in config.json")
        
        return config
    
    def _validate_environment(self):
        """Validate that required environment variables and tools are available."""
        # Check for Onshape API credentials
        required_env_vars = ['ONSHAPE_ACCESS_KEY', 'ONSHAPE_SECRET_KEY']
        missing_vars = [var for var in required_env_vars if not os.getenv(var)]
        
        if missing_vars:
            print("Warning: Missing environment variables:", ', '.join(missing_vars))
            print("Set these variables or use a .env file for Onshape API access.")
        
        # Check if onshape-to-robot is installed by trying to import it
        try:
            import onshape_to_robot
            if self.verbose:
                print(f"✓ onshape-to-robot version {getattr(onshape_to_robot, '__version__', 'unknown')} found")
        except ImportError:
            print("Error: onshape-to-robot is not installed.")
            print("Install it with: pip install onshape-to-robot")
            sys.exit(1)
    
    def _print_header(self):
        """Print pipeline header."""
        print("=" * 70)
        print("  Onshape to URDF Conversion Pipeline")
        print("=" * 70)
        print(f"Config file: {self.config_path}")
        print(f"Output directory: {self.output_dir}")
        print(f"Document ID: {self.config['documentId']}")
        print(f"Output format: {self.config.get('outputFormat', 'urdf')}")
        print("=" * 70)
        print()
    
    def _prepare_output_directory(self):
        """Create output directory structure."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        robot_name = self.config.get('assemblyName', 'robot').replace(' ', '_').lower()
        
        # Create timestamped directory
        self.output_dir = self.output_dir / f"{robot_name}_{timestamp}"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        print(f"Created output directory: {self.output_dir}")
        
        # Copy config to output directory
        output_config = self.output_dir / "config.json"
        with open(output_config, 'w') as f:
            json.dump(self.config, f, indent=2)
        print(f"Copied config to: {output_config}")
    
    def _run_conversion(self):
        """Run the onshape-to-robot conversion."""
        print("\nStarting conversion...")
        print("This may take several minutes depending on model complexity...")
        
        # Prepare command
        # Use an absolute path for the config file to avoid CWD-related issues after chdir
        config_file_abs = (self.output_dir / "config.json").resolve()
        
        cmd = ['onshape-to-robot', str(config_file_abs)]
        
        if self.verbose:
            cmd.append('--verbose')
        
        # Change to output directory for conversion
        original_dir = os.getcwd()
        os.chdir(self.output_dir)
        
        try:
            # Run conversion
            result = subprocess.run(
                cmd,
                capture_output=not self.verbose,
                text=True,
                timeout=600  # 10 minute timeout
            )
            
            if result.returncode != 0:
                print("Conversion failed!")
                if not self.verbose and result.stderr:
                    print("Error output:")
                    print(result.stderr)
                return False
            
            print("Conversion completed successfully!")
            return True
            
        except subprocess.TimeoutExpired:
            print("Conversion timed out (10 minutes)")
            return False
        except Exception as e:
            print(f"Conversion error: {e}")
            return False
        finally:
            os.chdir(original_dir)
    
    def _verify_output(self):
        """Verify that output files were generated."""
        print("\nVerifying output files...")
        
        output_format = self.config.get('outputFormat', 'urdf')
        expected_files = {
            'urdf': 'robot.urdf',
            'sdf': 'robot.sdf',
            'mjcf': 'robot.xml'
        }
        
        robot_file = self.output_dir / expected_files.get(output_format, 'robot.urdf')
        meshes_dir = self.output_dir / 'meshes'
        
        issues = []
        
        # Check robot file
        if robot_file.exists():
            size = robot_file.stat().st_size
            print(f"Found {output_format.upper()} file: {robot_file.name} ({size} bytes)")
        else:
            print(f"Missing {output_format.upper()} file: {robot_file.name}")
            issues.append(f"Missing {robot_file.name}")
        
        # Check meshes directory
        if meshes_dir.exists():
            mesh_files = list(meshes_dir.glob('*.stl')) + list(meshes_dir.glob('*.dae'))
            print(f"Found meshes directory with {len(mesh_files)} files")
        else:
            print("No meshes directory found (may be normal for simple geometries)")
        
        return len(issues) == 0

    def _emit_standalone_urdf(self, robot_file: Path) -> Path:
        """Create a standalone URDF with file:// absolute mesh paths.

        Args:
            robot_file: Path to the generated URDF inside the output directory

        Returns:
            Path to the standalone URDF file
        """
        meshes_dir = self.output_dir / 'meshes'
        assets_dir = self.output_dir / 'assets'

        # Build a map from mesh basename to absolute path
        basename_to_path: Dict[str, Path] = {}
        for dir_path in (meshes_dir, assets_dir):
            if not dir_path.exists():
                continue
            for mesh_path in dir_path.rglob('*'):
                if mesh_path.suffix.lower() in {'.stl', '.dae', '.obj'} and mesh_path.is_file():
                    basename_to_path[mesh_path.name] = mesh_path.resolve()

        # Read and parse the URDF
        with open(robot_file, 'r') as f:
            urdf_text = f.read()

        # Parse XML
        root = ET.fromstring(urdf_text)

        # Replace mesh URIs with file:// absolute paths using basename matching
        for mesh in root.findall('.//mesh'):
            uri = mesh.get('filename')
            if not uri:
                continue
            # Skip if already absolute file URI
            if uri.startswith('file://'):
                continue
            # Strategy 1: If it's a package:// URI, try to resolve relative to output dir
            if uri.startswith('package://'):
                # Example: package://robot_description/assets/base.stl -> assets/base.stl
                after_scheme = uri.split('://', 1)[1]
                parts = Path(after_scheme).parts
                relative_parts = parts[1:] if len(parts) > 1 else parts  # drop package name
                candidate_path = (self.output_dir.joinpath(*relative_parts)).resolve()
                if candidate_path.exists():
                    mesh.set('filename', f'file://{candidate_path.as_posix()}')
                    continue
            # Strategy 2: Fallback to basename lookup within known mesh dirs
            basename = Path(uri).name
            abs_path = basename_to_path.get(basename)
            if abs_path:
                mesh.set('filename', f'file://{abs_path.as_posix()}')

        # Serialize back to XML (URDF)
        standalone_path = self.output_dir / 'robot_standalone.urdf'
        urdf_string = ET.tostring(root, encoding='unicode')
        # Ensure XML declaration at top
        if not urdf_string.lstrip().startswith('<?xml'):
            urdf_string = '<?xml version="1.0" ?>\n' + urdf_string

        with open(standalone_path, 'w') as f:
            f.write(urdf_string)

        print(f"Created standalone URDF: {standalone_path}")
        return standalone_path
    
    def _print_summary(self, success):
        """Print conversion summary."""
        print("\n" + "=" * 70)
        if success:
            print("   Conversion Pipeline Completed Successfully!")
        else:
            print("   Conversion Pipeline Completed with Issues")
        print("=" * 70)
        print(f"\nOutput location: {self.output_dir.absolute()}")
        
        # List generated files
        print("\nGenerated files:")
        for item in sorted(self.output_dir.rglob('*')):
            if item.is_file():
                rel_path = item.relative_to(self.output_dir)
                size = item.stat().st_size
                print(f"  - {rel_path} ({size:,} bytes)")
        
        # Next steps
        print("\nNext Steps:")
        print("  1. Review the generated URDF file")
        print("  2. Test with: check_urdf robot.urdf")
        print("  3. Visualize with RViz or other tools")
        print("  4. Integrate into your robotics project")
        print()
    
    def convert(self):
        """Execute the full conversion pipeline."""
        try:
            self._print_header()
            self._prepare_output_directory()
            success = self._run_conversion()
            
            if success:
                success = self._verify_output()
                if success and self.standalone:
                    output_format = self.config.get('outputFormat', 'urdf')
                    if output_format != 'urdf':
                        print("Standalone mode currently only supported for URDF output")
                    else:
                        robot_file = self.output_dir / 'robot.urdf'
                        if robot_file.exists():
                            self._emit_standalone_urdf(robot_file)
                        else:
                            print("Expected robot.urdf not found; skipping standalone URDF generation")
            
            self._print_summary(success)
            
            return success
            
        except KeyboardInterrupt:
            print("\n\nConversion interrupted by user")
            return False
        except Exception as e:
            print(f"\nPipeline error: {e}")
            if self.verbose:
                import traceback
                traceback.print_exc()
            return False


def main():
    """Main entry point for the conversion script."""
    parser = argparse.ArgumentParser(
        description='Convert Onshape assemblies to URDF robot descriptions',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python convert.py --config config.json
  python convert.py --config examples/quadruped_config.json --output-dir robots/
  python convert.py --config config.json --verbose
        """
    )
    
    parser.add_argument(
        '--config',
        type=str,
        default='config.json',
        help='Path to configuration file (default: config.json)'
    )
    
    parser.add_argument(
        '--output-dir',
        type=str,
        default='output',
        help='Output directory for generated files (default: output/)'
    )
    
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable verbose output'
    )
    parser.add_argument(
        '--standalone',
        action='store_true',
        help='Also emit robot_standalone.urdf with file:// absolute mesh paths'
    )
    
    args = parser.parse_args()
    
    # Create converter and run
    converter = OnshapeToURDFConverter(
        config_path=args.config,
        output_dir=args.output_dir,
        verbose=args.verbose,
        standalone=args.standalone
    )
    
    success = converter.convert()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

