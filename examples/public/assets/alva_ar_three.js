/**
 * Example usage:
 *      import * as THREE from 'https://threejsfundamentals.org/threejs/resources/threejs/r132/build/three.module.js';
 *      import { AlvaAR } from 'alva_ar.js';
 *      import { AlvaAR } from 'alva_ar_three.js';
 *
 *      const alva = await AlvaAR.Initialize( ... );
 *      const applyPose = AlvaARConnectorTHREE.Initialize( THREE )
 *      const renderer = new THREE.WebGLRenderer( ... );
 *      const camera = new THREE.PerspectiveCamera( ... );
 *      const scene = new THREE.Scene();
 *      ...
 *
 *      function loop()
 *      {
 *          const imageData = ctx.getImageData( ... );
 *          const pose = alva.findCameraPose( imageData );
 *
 *          if( pose ) applyPose( pose, camera.quaternion, camera.position );
 *
 *          renderer.render( this.scene, this.camera );
 *      }
 */

class AlvaARConnectorTHREE
{
    static Initialize( THREE )
    {
        return ( pose, rotationQuaternion, translationVector ) =>
        {
            const m = new THREE.Matrix4();
            const q = new THREE.Quaternion();
            const t = new THREE.Vector3( pose[12], pose[13], pose[14] );
            const a = new THREE.Quaternion().setFromAxisAngle( new THREE.Vector3( 1, 0, 0 ), 0 );

            m.fromArray( pose );
            q.setFromRotationMatrix( m );
            q.multiply( a );

            ( rotationQuaternion !== null ) && rotationQuaternion.set( -q.x, q.y, q.z, q.w );
            ( translationVector !== null ) && translationVector.set( t.x, -t.y, -t.z );
        }
    }
}

export { AlvaARConnectorTHREE };
