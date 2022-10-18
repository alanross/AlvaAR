const singletonEnforcer = Symbol( 'Singleton Enforcer' );

class StatsTimer
{
    constructor( bufferSize = 30 )
    {
        this.t = 0;
        this.delta = 0;
        this.avg = new Array( bufferSize ).fill( 0 );
        this.idx = 0;
    }

    start()
    {
        this.t = performance.now();
    };

    stop()
    {
        this.delta = ~~( ( performance.now() - this.t ) + 0.5 );
        this.avg[this.idx] = this.delta;
        this.idx = ( this.idx + 1 ) % this.avg.length;
    };

    reset()
    {
        this.delta = 0;
    };

    getElapsedTime()
    {
        return this.delta;
    };

    getElapsedAverageTime()
    {
        return ~~( this.avg.reduce( ( acc, v ) => acc + ( v / this.avg.length ), 0 ) + 0.5 );
    };
}

class StatsBuffer
{
    constructor( size )
    {
        this.begin = 0;
        this.end = -1;
        this.numItems = 0;
        this.arrSize = size;
        this.arr = new Int32Array( size );
    }

    push( item )
    {
        if( this.numItems < this.arrSize )
        {
            this.end++;
            this.arr[this.end] = item;
            this.numItems++;
        }
        else
        {
            this.end = ( this.end + 1 ) % this.arrSize;
            this.begin = ( this.begin + 1 ) % this.arrSize;

            this.arr[this.end] = item;
        }
    };

    getAt( index )
    {
        return this.arr[( this.begin + index ) % this.arrSize];
    };

    size()
    {
        return this.numItems;
    };
}

class Stats
{
    constructor( enforcer )
    {
        if( enforcer !== singletonEnforcer )
        {
            throw new Error();
        }

        if( !Stats.instance )
        {
            Stats.instance = this;
        }

        this.fps = 0.0;
        this.frame = 0;
        this.timers = [];
        this.timer = new StatsTimer();
        this.buffer = new StatsBuffer( 16 );
        this.fpss = new Array( 50 ).fill( 0 );

        this.mbInfoAvailable = ( performance && performance.memory && performance.memory.totalJSHeapSize );
        this.mbSize = Math.pow( 1000, 2 );
        this.mb = 0;

        this.el = document.createElement( 'div' );
        this.el.style = `
            background:rgba(255,255,255,0.8); 
            position:absolute;
            top:0px; 
            left:0px; 
            display:block; 
            min-width:80px; 
            color:black; 
            font: 10px Arial, sans-serif;
            padding:5px;
            `;

        return Stats.instance;
    }

    add( taskName )
    {
        this.timers.push( [taskName, new StatsTimer()] );
    };

    next()
    {
        ++this.frame;

        this.timers.forEach( o => o[1].reset() );

        if( this.frame > 0 )
        {
            this.timer.stop();
            this.buffer.push( this.timer.getElapsedTime() );

            let size = this.buffer.size();
            let sum = 0;

            for( let i = 0; i < size; ++i )
            {
                sum += this.buffer.getAt( i );
            }

            this.fps = size / sum * 1000;
            this.fpss[this.frame % this.fpss.length] = ~~this.fps;
            this.timer.start();

        }

        if( this.mbInfoAvailable )
        {
            this.mb = performance.memory.usedJSHeapSize;
        }
    };

    getTask( taskName )
    {
        return this.timers.find( pair => ( pair[0] === taskName ) );
    };

    start( taskName )
    {
        this.getTask( taskName )[1].start();
    };

    stop( taskName )
    {
        this.getTask( taskName )[1].stop();
    };

    info()
    {
        let str = `FPS: ${ ~~this.fps } (${ Math.min( ...this.fpss ) } - ${ Math.max( ...this.fpss ) })`;

        this.timers.forEach( o => str += `\n${ o[0] } : ${ o[1].getElapsedTime() }ms` );

        if( this.mbInfoAvailable )
        {
            str += `\nMemory: ${ ( this.mb / this.mbSize ).toFixed( 2 ) }MB`;
        }

        return str;
    };

    render( info = null )
    {
        let str = `<b>FPS: ${ ~~this.fps } (${ Math.min( ...this.fpss ) } - ${ Math.max( ...this.fpss ) })</b>`;

        this.timers.forEach( o => str += `<br/>${ o[0] } : ${ o[1].getElapsedAverageTime() }ms` );

        if( this.mbInfoAvailable )
        {
            str += `<br/>Memory: ${ ( this.mb / this.mbSize ).toFixed( 2 ) }MB`;
        }

        if( info )
        {
            str += '<br/>' + info;
        }

        this.el.innerHTML = str;
    };
}

const instance = new Stats( singletonEnforcer );

export { instance as Stats };
