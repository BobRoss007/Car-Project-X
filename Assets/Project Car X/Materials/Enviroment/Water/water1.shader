// Upgrade NOTE: replaced '_Object2World' with 'unity_ObjectToWorld'
// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

// Shader created with Shader Forge v1.30 
// Shader Forge (c) Neat Corporation / Joachim Holmer - http://www.acegikmo.com/shaderforge/
// Note: Manually altering this data may prevent you from opening it in Shader Forge
/*SF_DATA;ver:1.30;sub:START;pass:START;ps:flbk:,iptp:0,cusa:False,bamd:0,lico:1,lgpr:1,limd:3,spmd:0,trmd:0,grmd:0,uamb:True,mssp:True,bkdf:True,hqlp:False,rprd:True,enco:False,rmgx:True,rpth:0,vtps:0,hqsc:True,nrmq:1,nrsp:0,vomd:0,spxs:False,tesm:0,olmd:1,culm:0,bsrc:3,bdst:7,dpts:2,wrdp:False,dith:0,rfrpo:True,rfrpn:Refraction,coma:15,ufog:True,aust:True,igpj:True,qofs:0,qpre:3,rntp:2,fgom:False,fgoc:False,fgod:False,fgor:False,fgmd:0,fgcr:0.5,fgcg:0.5,fgcb:0.5,fgca:1,fgde:0.01,fgrn:0,fgrf:300,stcl:False,stva:128,stmr:255,stmw:255,stcp:6,stps:0,stfa:0,stfz:0,ofsf:0,ofsu:0,f2p0:False,fnsp:False,fnfb:False;n:type:ShaderForge.SFN_Final,id:2865,x:32719,y:32712,varname:node_2865,prsc:2|diff-5363-OUT,spec-3669-OUT,gloss-1813-OUT,normal-6576-OUT,spcocc-3010-OUT,alpha-4832-OUT,refract-2769-OUT;n:type:ShaderForge.SFN_Color,id:6665,x:31902,y:32532,ptovrint:False,ptlb:Color,ptin:_Color,varname:_Color,prsc:0,glob:False,taghide:False,taghdr:False,tagprd:False,tagnsco:False,tagnrm:False,c1:0.1492215,c2:0.2221692,c3:0.2941176,c4:1;n:type:ShaderForge.SFN_Tex2d,id:5964,x:29931,y:32987,varname:_norm1,prsc:0,ntxv:3,isnm:True|UVIN-2963-OUT,TEX-2326-TEX;n:type:ShaderForge.SFN_Slider,id:358,x:31951,y:32836,ptovrint:False,ptlb:Specular,ptin:_Specular,varname:_Specular,prsc:0,glob:False,taghide:False,taghdr:False,tagprd:False,tagnsco:False,tagnrm:False,min:0,cur:0,max:1;n:type:ShaderForge.SFN_Slider,id:1813,x:31951,y:32937,ptovrint:False,ptlb:Gloss,ptin:_Gloss,varname:_Gloss,prsc:0,glob:False,taghide:False,taghdr:False,tagprd:False,tagnsco:False,tagnrm:False,min:0,cur:0.8,max:1;n:type:ShaderForge.SFN_TexCoord,id:7332,x:29164,y:33302,varname:node_7332,prsc:2,uv:0;n:type:ShaderForge.SFN_Tex2dAsset,id:2326,x:29672,y:33136,ptovrint:False,ptlb:Normal,ptin:_Normal,varname:_Normal,glob:False,taghide:False,taghdr:False,tagprd:False,tagnsco:False,tagnrm:False,ntxv:3,isnm:True;n:type:ShaderForge.SFN_Tex2d,id:4356,x:29931,y:33183,varname:_norm2,prsc:0,ntxv:3,isnm:True|UVIN-947-OUT,TEX-2326-TEX;n:type:ShaderForge.SFN_Add,id:3671,x:29164,y:33126,varname:node_3671,prsc:2|A-1517-OUT,B-7332-UVOUT;n:type:ShaderForge.SFN_Vector1,id:1517,x:28973,y:33126,varname:node_1517,prsc:2,v1:0.03;n:type:ShaderForge.SFN_Add,id:2963,x:29402,y:33126,varname:node_2963,prsc:0|A-3625-OUT,B-3671-OUT;n:type:ShaderForge.SFN_Time,id:9627,x:28699,y:33021,varname:node_9627,prsc:0;n:type:ShaderForge.SFN_Multiply,id:3625,x:28973,y:32951,varname:node_3625,prsc:0|A-9627-T,B-2124-OUT;n:type:ShaderForge.SFN_Multiply,id:8556,x:29588,y:32827,varname:node_8556,prsc:2|A-2175-OUT,B-9627-T,C-1748-OUT;n:type:ShaderForge.SFN_Pi,id:2175,x:29435,y:32827,varname:node_2175,prsc:2;n:type:ShaderForge.SFN_Sin,id:102,x:29756,y:32827,varname:node_102,prsc:2|IN-8556-OUT;n:type:ShaderForge.SFN_Lerp,id:1712,x:30325,y:33064,varname:node_1712,prsc:0|A-8984-OUT,B-8695-OUT,T-3083-OUT;n:type:ShaderForge.SFN_Append,id:8984,x:30116,y:32987,varname:node_8984,prsc:2|A-5964-R,B-5964-G;n:type:ShaderForge.SFN_Append,id:8695,x:30116,y:33183,varname:node_8695,prsc:2|A-4356-R,B-4356-G;n:type:ShaderForge.SFN_Append,id:2493,x:30731,y:33064,varname:node_2493,prsc:2|A-7756-OUT,B-4744-OUT;n:type:ShaderForge.SFN_Vector1,id:4744,x:30563,y:33206,varname:node_4744,prsc:2,v1:1;n:type:ShaderForge.SFN_Add,id:947,x:29402,y:33300,varname:node_947,prsc:2|A-7351-OUT,B-7332-UVOUT;n:type:ShaderForge.SFN_ValueProperty,id:1748,x:29402,y:32956,ptovrint:False,ptlb:WaveAltSpeed,ptin:_WaveAltSpeed,varname:_WaveAltSpeed,prsc:2,glob:False,taghide:False,taghdr:False,tagprd:False,tagnsco:False,tagnrm:False,v1:1;n:type:ShaderForge.SFN_RemapRange,id:8985,x:29931,y:32827,varname:node_8985,prsc:2,frmn:-1,frmx:1,tomn:0,tomx:1|IN-102-OUT;n:type:ShaderForge.SFN_Clamp01,id:3083,x:30116,y:32827,varname:node_3083,prsc:2|IN-8985-OUT;n:type:ShaderForge.SFN_Vector4Property,id:5677,x:28458,y:33279,ptovrint:False,ptlb:WaterSpeed,ptin:_WaterSpeed,varname:_WaterSpeed,prsc:2,glob:False,taghide:False,taghdr:False,tagprd:False,tagnsco:False,tagnrm:False,v1:0.003,v2:0,v3:0.001,v4:0;n:type:ShaderForge.SFN_Append,id:2124,x:28713,y:33263,varname:node_2124,prsc:2|A-5677-X,B-5677-Y;n:type:ShaderForge.SFN_Append,id:9485,x:28713,y:33430,varname:node_9485,prsc:2|A-5677-Z,B-5677-W;n:type:ShaderForge.SFN_Multiply,id:7351,x:28973,y:33259,varname:node_7351,prsc:2|A-9627-T,B-9485-OUT;n:type:ShaderForge.SFN_Lerp,id:2769,x:32122,y:33260,varname:node_2769,prsc:2|A-4204-OUT,B-2294-OUT,T-8052-OUT;n:type:ShaderForge.SFN_Vector1,id:4204,x:31938,y:33202,varname:node_4204,prsc:2,v1:0;n:type:ShaderForge.SFN_Slider,id:9241,x:31572,y:33368,ptovrint:False,ptlb:Refraction,ptin:_Refraction,varname:_Refraction,prsc:0,glob:False,taghide:False,taghdr:False,tagprd:False,tagnsco:False,tagnrm:False,min:0,cur:1,max:1;n:type:ShaderForge.SFN_Fresnel,id:8544,x:31600,y:33754,varname:node_8544,prsc:0|NRM-496-OUT,EXP-3781-OUT;n:type:ShaderForge.SFN_NormalVector,id:496,x:31422,y:33754,prsc:2,pt:True;n:type:ShaderForge.SFN_ValueProperty,id:3781,x:31422,y:33919,ptovrint:False,ptlb:Fresnel Pinch,ptin:_FresnelPinch,varname:_FresnelPinch,prsc:0,glob:False,taghide:False,taghdr:False,tagprd:False,tagnsco:False,tagnrm:False,v1:1;n:type:ShaderForge.SFN_DepthBlend,id:4138,x:30079,y:34129,varname:node_4138,prsc:0|DIST-4928-OUT;n:type:ShaderForge.SFN_ValueProperty,id:4928,x:29918,y:34129,ptovrint:False,ptlb:Depth,ptin:_Depth,varname:_Depth,prsc:0,glob:False,taghide:False,taghdr:False,tagprd:False,tagnsco:False,tagnrm:False,v1:2;n:type:ShaderForge.SFN_OneMinus,id:3322,x:30254,y:34129,varname:node_3322,prsc:2|IN-4138-OUT;n:type:ShaderForge.SFN_Set,id:3023,x:30440,y:34129,varname:waterFog,prsc:0|IN-4138-OUT;n:type:ShaderForge.SFN_Get,id:5820,x:32068,y:32714,varname:node_5820,prsc:2|IN-3023-OUT;n:type:ShaderForge.SFN_Add,id:4275,x:31760,y:33754,varname:node_4275,prsc:2|A-8544-OUT,B-3846-OUT;n:type:ShaderForge.SFN_Get,id:3846,x:31531,y:33650,varname:node_3846,prsc:0|IN-3023-OUT;n:type:ShaderForge.SFN_Clamp01,id:4388,x:31922,y:33754,varname:node_4388,prsc:0|IN-4275-OUT;n:type:ShaderForge.SFN_Set,id:5597,x:32063,y:32601,varname:colAlpha,prsc:0|IN-6665-A;n:type:ShaderForge.SFN_Get,id:8797,x:31901,y:33687,varname:node_8797,prsc:0|IN-5597-OUT;n:type:ShaderForge.SFN_Multiply,id:8052,x:31938,y:33332,varname:node_8052,prsc:2|A-9241-OUT,B-3846-OUT,C-3033-OUT;n:type:ShaderForge.SFN_OneMinus,id:565,x:32245,y:32714,varname:node_565,prsc:2|IN-5820-OUT;n:type:ShaderForge.SFN_Lerp,id:6812,x:32133,y:33754,varname:node_6812,prsc:2|A-8797-OUT,B-8356-OUT,T-4388-OUT;n:type:ShaderForge.SFN_Vector1,id:8356,x:31922,y:33885,varname:node_8356,prsc:2,v1:1;n:type:ShaderForge.SFN_RemapRange,id:600,x:31938,y:33476,varname:node_600,prsc:2,frmn:0,frmx:0.1,tomn:0,tomx:1|IN-3846-OUT;n:type:ShaderForge.SFN_Clamp01,id:8444,x:32122,y:33476,varname:node_8444,prsc:2|IN-600-OUT;n:type:ShaderForge.SFN_Multiply,id:4832,x:32319,y:33577,varname:node_4832,prsc:2|A-8444-OUT,B-6812-OUT,C-7820-OUT;n:type:ShaderForge.SFN_Depth,id:8046,x:31480,y:34551,varname:node_8046,prsc:2;n:type:ShaderForge.SFN_Divide,id:5453,x:31657,y:34551,varname:node_5453,prsc:2|A-8046-OUT,B-5311-OUT;n:type:ShaderForge.SFN_ValueProperty,id:5311,x:31480,y:34700,ptovrint:False,ptlb:Water Distance,ptin:_WaterDistance,varname:_WaterDistance,prsc:2,glob:False,taghide:False,taghdr:False,tagprd:False,tagnsco:False,tagnrm:False,v1:50;n:type:ShaderForge.SFN_Clamp01,id:9548,x:31994,y:34551,varname:node_9548,prsc:2|IN-6321-OUT;n:type:ShaderForge.SFN_OneMinus,id:4470,x:32158,y:34551,varname:node_4470,prsc:2|IN-9548-OUT;n:type:ShaderForge.SFN_Set,id:4185,x:32310,y:34551,varname:distanceMult,prsc:2|IN-4470-OUT;n:type:ShaderForge.SFN_Get,id:7820,x:32111,y:33662,varname:node_7820,prsc:2|IN-4185-OUT;n:type:ShaderForge.SFN_Get,id:3010,x:32245,y:32887,varname:node_3010,prsc:0|IN-4185-OUT;n:type:ShaderForge.SFN_Multiply,id:3669,x:32423,y:32851,varname:node_3669,prsc:2|A-358-OUT,B-3010-OUT;n:type:ShaderForge.SFN_Multiply,id:5363,x:32472,y:32590,varname:node_5363,prsc:2|A-6665-RGB,B-565-OUT;n:type:ShaderForge.SFN_RemapRange,id:6321,x:31827,y:34551,varname:node_6321,prsc:2,frmn:0.33,frmx:1,tomn:0,tomx:1|IN-5453-OUT;n:type:ShaderForge.SFN_Depth,id:1179,x:31443,y:34179,varname:node_1179,prsc:2;n:type:ShaderForge.SFN_Divide,id:5899,x:31637,y:34179,varname:node_5899,prsc:2|A-1179-OUT,B-2029-OUT;n:type:ShaderForge.SFN_ValueProperty,id:2029,x:31443,y:34347,ptovrint:False,ptlb:Refraction Distance,ptin:_RefractionDistance,varname:_RefractionDistance,prsc:2,glob:False,taghide:False,taghdr:False,tagprd:False,tagnsco:False,tagnrm:False,v1:10;n:type:ShaderForge.SFN_RemapRange,id:5686,x:31806,y:34179,varname:node_5686,prsc:2,frmn:0.33,frmx:1,tomn:0,tomx:1|IN-5899-OUT;n:type:ShaderForge.SFN_Clamp01,id:6948,x:31976,y:34179,varname:node_6948,prsc:2|IN-5686-OUT;n:type:ShaderForge.SFN_OneMinus,id:6144,x:32159,y:34179,varname:node_6144,prsc:2|IN-6948-OUT;n:type:ShaderForge.SFN_Set,id:7393,x:32331,y:34179,varname:refracDist,prsc:2|IN-6144-OUT;n:type:ShaderForge.SFN_Get,id:3033,x:31531,y:33542,varname:node_3033,prsc:2|IN-7393-OUT;n:type:ShaderForge.SFN_Set,id:2471,x:30563,y:32950,varname:normalRG,prsc:0|IN-1712-OUT;n:type:ShaderForge.SFN_Lerp,id:7756,x:30563,y:33064,varname:node_7756,prsc:2|A-4479-OUT,B-1712-OUT,T-828-OUT;n:type:ShaderForge.SFN_Vector2,id:4479,x:30325,y:32950,varname:node_4479,prsc:2,v1:0,v2:0;n:type:ShaderForge.SFN_Slider,id:828,x:30116,y:33335,ptovrint:False,ptlb:Normal Strength,ptin:_NormalStrength,varname:_NormalStrength,prsc:2,glob:False,taghide:False,taghdr:False,tagprd:False,tagnsco:False,tagnrm:False,min:0,cur:1,max:1;n:type:ShaderForge.SFN_Get,id:2294,x:31917,y:33271,varname:node_2294,prsc:2|IN-2471-OUT;n:type:ShaderForge.SFN_Set,id:1682,x:30895,y:33064,varname:normalFinal,prsc:0|IN-2493-OUT;n:type:ShaderForge.SFN_Get,id:6576,x:32423,y:32791,varname:node_6576,prsc:2|IN-1682-OUT;proporder:6665-3781-358-1813-2326-828-9241-2029-5677-1748-4928-5311;pass:END;sub:END;*/

Shader "Shader Forge/World/water1" {
    Properties {
        _Color ("Color", Color) = (0.1492215,0.2221692,0.2941176,1)
        _FresnelPinch ("Fresnel Pinch", Float ) = 1
        _Specular ("Specular", Range(0, 1)) = 0
        _Gloss ("Gloss", Range(0, 1)) = 0.8
        _Normal ("Normal", 2D) = "bump" {}
        _NormalStrength ("Normal Strength", Range(0, 1)) = 1
        _Refraction ("Refraction", Range(0, 1)) = 1
        _RefractionDistance ("Refraction Distance", Float ) = 10
        _WaterSpeed ("WaterSpeed", Vector) = (0.003,0,0.001,0)
        _WaveAltSpeed ("WaveAltSpeed", Float ) = 1
        _Depth ("Depth", Float ) = 2
        _WaterDistance ("Water Distance", Float ) = 50
        [HideInInspector]_Cutoff ("Alpha cutoff", Range(0,1)) = 0.5
    }
    SubShader {
        Tags {
            "IgnoreProjector"="True"
            "Queue"="Transparent"
            "RenderType"="Transparent"
        }
        GrabPass{ }
        Pass {
            Name "FORWARD"
            Tags {
                "LightMode"="ForwardBase"
            }
            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #define UNITY_PASS_FORWARDBASE
            #define SHOULD_SAMPLE_SH ( defined (LIGHTMAP_OFF) && defined(DYNAMICLIGHTMAP_OFF) )
            #define _GLOSSYENV 1
            #include "UnityCG.cginc"
            #include "Lighting.cginc"
            #include "UnityPBSLighting.cginc"
            #include "UnityStandardBRDF.cginc"
            #pragma multi_compile_fwdbase
            #pragma multi_compile LIGHTMAP_OFF LIGHTMAP_ON
            #pragma multi_compile DIRLIGHTMAP_OFF DIRLIGHTMAP_COMBINED DIRLIGHTMAP_SEPARATE
            #pragma multi_compile DYNAMICLIGHTMAP_OFF DYNAMICLIGHTMAP_ON
            #pragma multi_compile_fog
            #pragma exclude_renderers gles3 metal d3d11_9x xbox360 xboxone ps3 ps4 psp2 
            #pragma target 3.0
            uniform sampler2D _GrabTexture;
            uniform sampler2D _CameraDepthTexture;
            uniform float4 _TimeEditor;
            uniform fixed4 _Color;
            uniform fixed _Specular;
            uniform fixed _Gloss;
            uniform sampler2D _Normal; uniform float4 _Normal_ST;
            uniform float _WaveAltSpeed;
            uniform float4 _WaterSpeed;
            uniform fixed _Refraction;
            uniform fixed _FresnelPinch;
            uniform fixed _Depth;
            uniform float _WaterDistance;
            uniform float _RefractionDistance;
            uniform float _NormalStrength;
            struct VertexInput {
                float4 vertex : POSITION;
                float3 normal : NORMAL;
                float4 tangent : TANGENT;
                float2 texcoord0 : TEXCOORD0;
                float2 texcoord1 : TEXCOORD1;
                float2 texcoord2 : TEXCOORD2;
            };
            struct VertexOutput {
                float4 pos : SV_POSITION;
                float2 uv0 : TEXCOORD0;
                float2 uv1 : TEXCOORD1;
                float2 uv2 : TEXCOORD2;
                float4 posWorld : TEXCOORD3;
                float3 normalDir : TEXCOORD4;
                float3 tangentDir : TEXCOORD5;
                float3 bitangentDir : TEXCOORD6;
                float4 screenPos : TEXCOORD7;
                float4 projPos : TEXCOORD8;
                UNITY_FOG_COORDS(9)
                #if defined(LIGHTMAP_ON) || defined(UNITY_SHOULD_SAMPLE_SH)
                    float4 ambientOrLightmapUV : TEXCOORD10;
                #endif
            };
            VertexOutput vert (VertexInput v) {
                VertexOutput o = (VertexOutput)0;
                o.uv0 = v.texcoord0;
                o.uv1 = v.texcoord1;
                o.uv2 = v.texcoord2;
                #ifdef LIGHTMAP_ON
                    o.ambientOrLightmapUV.xy = v.texcoord1.xy * unity_LightmapST.xy + unity_LightmapST.zw;
                    o.ambientOrLightmapUV.zw = 0;
                #elif UNITY_SHOULD_SAMPLE_SH
                #endif
                #ifdef DYNAMICLIGHTMAP_ON
                    o.ambientOrLightmapUV.zw = v.texcoord2.xy * unity_DynamicLightmapST.xy + unity_DynamicLightmapST.zw;
                #endif
                o.normalDir = UnityObjectToWorldNormal(v.normal);
                o.tangentDir = normalize( mul( unity_ObjectToWorld, float4( v.tangent.xyz, 0.0 ) ).xyz );
                o.bitangentDir = normalize(cross(o.normalDir, o.tangentDir) * v.tangent.w);
                o.posWorld = mul(unity_ObjectToWorld, v.vertex);
                float3 lightColor = _LightColor0.rgb;
                o.pos = UnityObjectToClipPos(v.vertex );
                UNITY_TRANSFER_FOG(o,o.pos);
                o.projPos = ComputeScreenPos (o.pos);
                COMPUTE_EYEDEPTH(o.projPos.z);
                o.screenPos = o.pos;
                return o;
            }
            float4 frag(VertexOutput i) : COLOR {
                #if UNITY_UV_STARTS_AT_TOP
                    float grabSign = -_ProjectionParams.x;
                #else
                    float grabSign = _ProjectionParams.x;
                #endif
                i.normalDir = normalize(i.normalDir);
                i.screenPos = float4( i.screenPos.xy / i.screenPos.w, 0, 0 );
                i.screenPos.y *= _ProjectionParams.x;
                float3x3 tangentTransform = float3x3( i.tangentDir, i.bitangentDir, i.normalDir);
                float3 viewDirection = normalize(_WorldSpaceCameraPos.xyz - i.posWorld.xyz);
                fixed4 node_9627 = _Time + _TimeEditor;
                fixed2 node_2963 = ((node_9627.g*float2(_WaterSpeed.r,_WaterSpeed.g))+(0.03+i.uv0));
                fixed3 _norm1 = UnpackNormal(tex2D(_Normal,TRANSFORM_TEX(node_2963, _Normal)));
                float2 node_947 = ((node_9627.g*float2(_WaterSpeed.b,_WaterSpeed.a))+i.uv0);
                fixed3 _norm2 = UnpackNormal(tex2D(_Normal,TRANSFORM_TEX(node_947, _Normal)));
                fixed2 node_1712 = lerp(float2(_norm1.r,_norm1.g),float2(_norm2.r,_norm2.g),saturate((sin((3.141592654*node_9627.g*_WaveAltSpeed))*0.5+0.5)));
                fixed3 normalFinal = float3(lerp(float2(0,0),node_1712,_NormalStrength),1.0);
                float3 normalLocal = normalFinal;
                float3 normalDirection = normalize(mul( normalLocal, tangentTransform )); // Perturbed normals
                float3 viewReflectDirection = reflect( -viewDirection, normalDirection );
                float sceneZ = max(0,LinearEyeDepth (UNITY_SAMPLE_DEPTH(tex2Dproj(_CameraDepthTexture, UNITY_PROJ_COORD(i.projPos)))) - _ProjectionParams.g);
                float partZ = max(0,i.projPos.z - _ProjectionParams.g);
                float node_4204 = 0.0;
                fixed2 normalRG = node_1712;
                fixed node_4138 = saturate((sceneZ-partZ)/_Depth);
                fixed waterFog = node_4138;
                fixed node_3846 = waterFog;
                float refracDist = (1.0 - saturate(((partZ/_RefractionDistance)*1.492537+-0.4925373)));
                float2 sceneUVs = float2(1,grabSign)*i.screenPos.xy*0.5+0.5 + lerp(float2(node_4204,node_4204),normalRG,(_Refraction*node_3846*refracDist));
                float4 sceneColor = tex2D(_GrabTexture, sceneUVs);
                float3 lightDirection = normalize(_WorldSpaceLightPos0.xyz);
                float3 lightColor = _LightColor0.rgb;
                float3 halfDirection = normalize(viewDirection+lightDirection);
////// Lighting:
                float attenuation = 1;
                float3 attenColor = attenuation * _LightColor0.xyz;
                float Pi = 3.141592654;
                float InvPi = 0.31830988618;
///////// Gloss:
                float gloss = _Gloss;
                float specPow = exp2( gloss * 10.0+1.0);
/////// GI Data:
                UnityLight light;
                #ifdef LIGHTMAP_OFF
                    light.color = lightColor;
                    light.dir = lightDirection;
                    light.ndotl = LambertTerm (normalDirection, light.dir);
                #else
                    light.color = half3(0.f, 0.f, 0.f);
                    light.ndotl = 0.0f;
                    light.dir = half3(0.f, 0.f, 0.f);
                #endif
                UnityGIInput d;
                d.light = light;
                d.worldPos = i.posWorld.xyz;
                d.worldViewDir = viewDirection;
                d.atten = attenuation;
                #if defined(LIGHTMAP_ON) || defined(DYNAMICLIGHTMAP_ON)
                    d.ambient = 0;
                    d.lightmapUV = i.ambientOrLightmapUV;
                #else
                    d.ambient = i.ambientOrLightmapUV;
                #endif
                d.boxMax[0] = unity_SpecCube0_BoxMax;
                d.boxMin[0] = unity_SpecCube0_BoxMin;
                d.probePosition[0] = unity_SpecCube0_ProbePosition;
                d.probeHDR[0] = unity_SpecCube0_HDR;
                d.boxMax[1] = unity_SpecCube1_BoxMax;
                d.boxMin[1] = unity_SpecCube1_BoxMin;
                d.probePosition[1] = unity_SpecCube1_ProbePosition;
                d.probeHDR[1] = unity_SpecCube1_HDR;
                Unity_GlossyEnvironmentData ugls_en_data;
                ugls_en_data.roughness = 1.0 - gloss;
                ugls_en_data.reflUVW = viewReflectDirection;
                UnityGI gi = UnityGlobalIllumination(d, 1, normalDirection, ugls_en_data );
                lightDirection = gi.light.dir;
                lightColor = gi.light.color;
////// Specular:
                float NdotL = max(0, dot( normalDirection, lightDirection ));
                float distanceMult = (1.0 - saturate(((partZ/_WaterDistance)*1.492537+-0.4925373)));
                fixed node_3010 = distanceMult;
                float3 specularAO = node_3010;
                float LdotH = max(0.0,dot(lightDirection, halfDirection));
                float node_3669 = (_Specular*node_3010);
                float3 specularColor = float3(node_3669,node_3669,node_3669);
                float specularMonochrome;
                float3 diffuseColor = (_Color.rgb*(1.0 - waterFog)); // Need this for specular when using metallic
                diffuseColor = EnergyConservationBetweenDiffuseAndSpecular(diffuseColor, specularColor, specularMonochrome);
                specularMonochrome = 1.0-specularMonochrome;
                float NdotV = max(0.0,dot( normalDirection, viewDirection ));
                float NdotH = max(0.0,dot( normalDirection, halfDirection ));
                float VdotH = max(0.0,dot( viewDirection, halfDirection ));
                float visTerm = SmithJointGGXVisibilityTerm( NdotL, NdotV, 1.0-gloss );
                float normTerm = max(0.0, GGXTerm(NdotH, 1.0-gloss));
                float specularPBL = (NdotL*visTerm*normTerm) * (UNITY_PI / 4);
                if (IsGammaSpace())
                    specularPBL = sqrt(max(1e-4h, specularPBL));
                specularPBL = max(0, specularPBL * NdotL);
                float3 directSpecular = (floor(attenuation) * _LightColor0.xyz)*specularPBL*FresnelTerm(specularColor, LdotH);
                half grazingTerm = saturate( gloss + specularMonochrome );
                float3 indirectSpecular = (gi.indirect.specular) * specularAO;
                indirectSpecular *= FresnelLerp (specularColor, grazingTerm, NdotV);
                float3 specular = (directSpecular + indirectSpecular);
/////// Diffuse:
                NdotL = max(0.0,dot( normalDirection, lightDirection ));
                half fd90 = 0.5 + 2 * LdotH * LdotH * (1-gloss);
                float nlPow5 = Pow5(1-NdotL);
                float nvPow5 = Pow5(1-NdotV);
                float3 directDiffuse = ((1 +(fd90 - 1)*nlPow5) * (1 + (fd90 - 1)*nvPow5) * NdotL) * attenColor;
                float3 indirectDiffuse = float3(0,0,0);
                indirectDiffuse += gi.indirect.diffuse;
                diffuseColor *= 1-specularMonochrome;
                float3 diffuse = (directDiffuse + indirectDiffuse) * diffuseColor;
/// Final Color:
                float3 finalColor = diffuse + specular;
                fixed colAlpha = _Color.a;
                fixed4 finalRGBA = fixed4(lerp(sceneColor.rgb, finalColor,(saturate((node_3846*10.0+0.0))*lerp(colAlpha,1.0,saturate((pow(1.0-max(0,dot(normalDirection, viewDirection)),_FresnelPinch)+node_3846)))*distanceMult)),1);
                UNITY_APPLY_FOG(i.fogCoord, finalRGBA);
                return finalRGBA;
            }
            ENDCG
        }
        Pass {
            Name "FORWARD_DELTA"
            Tags {
                "LightMode"="ForwardAdd"
            }
            Blend One One
            ZWrite Off
            
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #define UNITY_PASS_FORWARDADD
            #define SHOULD_SAMPLE_SH ( defined (LIGHTMAP_OFF) && defined(DYNAMICLIGHTMAP_OFF) )
            #define _GLOSSYENV 1
            #include "UnityCG.cginc"
            #include "AutoLight.cginc"
            #include "Lighting.cginc"
            #include "UnityPBSLighting.cginc"
            #include "UnityStandardBRDF.cginc"
            #pragma multi_compile_fwdadd
            #pragma multi_compile LIGHTMAP_OFF LIGHTMAP_ON
            #pragma multi_compile DIRLIGHTMAP_OFF DIRLIGHTMAP_COMBINED DIRLIGHTMAP_SEPARATE
            #pragma multi_compile DYNAMICLIGHTMAP_OFF DYNAMICLIGHTMAP_ON
            #pragma multi_compile_fog
            #pragma exclude_renderers gles3 metal d3d11_9x xbox360 xboxone ps3 ps4 psp2 
            #pragma target 3.0
            uniform sampler2D _GrabTexture;
            uniform sampler2D _CameraDepthTexture;
            uniform float4 _TimeEditor;
            uniform fixed4 _Color;
            uniform fixed _Specular;
            uniform fixed _Gloss;
            uniform sampler2D _Normal; uniform float4 _Normal_ST;
            uniform float _WaveAltSpeed;
            uniform float4 _WaterSpeed;
            uniform fixed _Refraction;
            uniform fixed _FresnelPinch;
            uniform fixed _Depth;
            uniform float _WaterDistance;
            uniform float _RefractionDistance;
            uniform float _NormalStrength;
            struct VertexInput {
                float4 vertex : POSITION;
                float3 normal : NORMAL;
                float4 tangent : TANGENT;
                float2 texcoord0 : TEXCOORD0;
                float2 texcoord1 : TEXCOORD1;
                float2 texcoord2 : TEXCOORD2;
            };
            struct VertexOutput {
                float4 pos : SV_POSITION;
                float2 uv0 : TEXCOORD0;
                float2 uv1 : TEXCOORD1;
                float2 uv2 : TEXCOORD2;
                float4 posWorld : TEXCOORD3;
                float3 normalDir : TEXCOORD4;
                float3 tangentDir : TEXCOORD5;
                float3 bitangentDir : TEXCOORD6;
                float4 screenPos : TEXCOORD7;
                float4 projPos : TEXCOORD8;
                LIGHTING_COORDS(9,10)
                UNITY_FOG_COORDS(11)
            };
            VertexOutput vert (VertexInput v) {
                VertexOutput o = (VertexOutput)0;
                o.uv0 = v.texcoord0;
                o.uv1 = v.texcoord1;
                o.uv2 = v.texcoord2;
                o.normalDir = UnityObjectToWorldNormal(v.normal);
                o.tangentDir = normalize( mul( unity_ObjectToWorld, float4( v.tangent.xyz, 0.0 ) ).xyz );
                o.bitangentDir = normalize(cross(o.normalDir, o.tangentDir) * v.tangent.w);
                o.posWorld = mul(unity_ObjectToWorld, v.vertex);
                float3 lightColor = _LightColor0.rgb;
                o.pos = UnityObjectToClipPos(v.vertex );
                UNITY_TRANSFER_FOG(o,o.pos);
                o.projPos = ComputeScreenPos (o.pos);
                COMPUTE_EYEDEPTH(o.projPos.z);
                o.screenPos = o.pos;
                TRANSFER_VERTEX_TO_FRAGMENT(o)
                return o;
            }
            float4 frag(VertexOutput i) : COLOR {
                #if UNITY_UV_STARTS_AT_TOP
                    float grabSign = -_ProjectionParams.x;
                #else
                    float grabSign = _ProjectionParams.x;
                #endif
                i.normalDir = normalize(i.normalDir);
                i.screenPos = float4( i.screenPos.xy / i.screenPos.w, 0, 0 );
                i.screenPos.y *= _ProjectionParams.x;
                float3x3 tangentTransform = float3x3( i.tangentDir, i.bitangentDir, i.normalDir);
                float3 viewDirection = normalize(_WorldSpaceCameraPos.xyz - i.posWorld.xyz);
                fixed4 node_9627 = _Time + _TimeEditor;
                fixed2 node_2963 = ((node_9627.g*float2(_WaterSpeed.r,_WaterSpeed.g))+(0.03+i.uv0));
                fixed3 _norm1 = UnpackNormal(tex2D(_Normal,TRANSFORM_TEX(node_2963, _Normal)));
                float2 node_947 = ((node_9627.g*float2(_WaterSpeed.b,_WaterSpeed.a))+i.uv0);
                fixed3 _norm2 = UnpackNormal(tex2D(_Normal,TRANSFORM_TEX(node_947, _Normal)));
                fixed2 node_1712 = lerp(float2(_norm1.r,_norm1.g),float2(_norm2.r,_norm2.g),saturate((sin((3.141592654*node_9627.g*_WaveAltSpeed))*0.5+0.5)));
                fixed3 normalFinal = float3(lerp(float2(0,0),node_1712,_NormalStrength),1.0);
                float3 normalLocal = normalFinal;
                float3 normalDirection = normalize(mul( normalLocal, tangentTransform )); // Perturbed normals
                float sceneZ = max(0,LinearEyeDepth (UNITY_SAMPLE_DEPTH(tex2Dproj(_CameraDepthTexture, UNITY_PROJ_COORD(i.projPos)))) - _ProjectionParams.g);
                float partZ = max(0,i.projPos.z - _ProjectionParams.g);
                float node_4204 = 0.0;
                fixed2 normalRG = node_1712;
                fixed node_4138 = saturate((sceneZ-partZ)/_Depth);
                fixed waterFog = node_4138;
                fixed node_3846 = waterFog;
                float refracDist = (1.0 - saturate(((partZ/_RefractionDistance)*1.492537+-0.4925373)));
                float2 sceneUVs = float2(1,grabSign)*i.screenPos.xy*0.5+0.5 + lerp(float2(node_4204,node_4204),normalRG,(_Refraction*node_3846*refracDist));
                float4 sceneColor = tex2D(_GrabTexture, sceneUVs);
                float3 lightDirection = normalize(lerp(_WorldSpaceLightPos0.xyz, _WorldSpaceLightPos0.xyz - i.posWorld.xyz,_WorldSpaceLightPos0.w));
                float3 lightColor = _LightColor0.rgb;
                float3 halfDirection = normalize(viewDirection+lightDirection);
////// Lighting:
                float attenuation = LIGHT_ATTENUATION(i);
                float3 attenColor = attenuation * _LightColor0.xyz;
                float Pi = 3.141592654;
                float InvPi = 0.31830988618;
///////// Gloss:
                float gloss = _Gloss;
                float specPow = exp2( gloss * 10.0+1.0);
////// Specular:
                float NdotL = max(0, dot( normalDirection, lightDirection ));
                float LdotH = max(0.0,dot(lightDirection, halfDirection));
                float distanceMult = (1.0 - saturate(((partZ/_WaterDistance)*1.492537+-0.4925373)));
                fixed node_3010 = distanceMult;
                float node_3669 = (_Specular*node_3010);
                float3 specularColor = float3(node_3669,node_3669,node_3669);
                float specularMonochrome;
                float3 diffuseColor = (_Color.rgb*(1.0 - waterFog)); // Need this for specular when using metallic
                diffuseColor = EnergyConservationBetweenDiffuseAndSpecular(diffuseColor, specularColor, specularMonochrome);
                specularMonochrome = 1.0-specularMonochrome;
                float NdotV = max(0.0,dot( normalDirection, viewDirection ));
                float NdotH = max(0.0,dot( normalDirection, halfDirection ));
                float VdotH = max(0.0,dot( viewDirection, halfDirection ));
                float visTerm = SmithJointGGXVisibilityTerm( NdotL, NdotV, 1.0-gloss );
                float normTerm = max(0.0, GGXTerm(NdotH, 1.0-gloss));
                float specularPBL = (NdotL*visTerm*normTerm) * (UNITY_PI / 4);
                if (IsGammaSpace())
                    specularPBL = sqrt(max(1e-4h, specularPBL));
                specularPBL = max(0, specularPBL * NdotL);
                float3 directSpecular = attenColor*specularPBL*FresnelTerm(specularColor, LdotH);
                float3 specular = directSpecular;
/////// Diffuse:
                NdotL = max(0.0,dot( normalDirection, lightDirection ));
                half fd90 = 0.5 + 2 * LdotH * LdotH * (1-gloss);
                float nlPow5 = Pow5(1-NdotL);
                float nvPow5 = Pow5(1-NdotV);
                float3 directDiffuse = ((1 +(fd90 - 1)*nlPow5) * (1 + (fd90 - 1)*nvPow5) * NdotL) * attenColor;
                diffuseColor *= 1-specularMonochrome;
                float3 diffuse = directDiffuse * diffuseColor;
/// Final Color:
                float3 finalColor = diffuse + specular;
                fixed colAlpha = _Color.a;
                fixed4 finalRGBA = fixed4(finalColor * (saturate((node_3846*10.0+0.0))*lerp(colAlpha,1.0,saturate((pow(1.0-max(0,dot(normalDirection, viewDirection)),_FresnelPinch)+node_3846)))*distanceMult),0);
                UNITY_APPLY_FOG(i.fogCoord, finalRGBA);
                return finalRGBA;
            }
            ENDCG
        }
        Pass {
            Name "Meta"
            Tags {
                "LightMode"="Meta"
            }
            Cull Off
            
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #define UNITY_PASS_META 1
            #define SHOULD_SAMPLE_SH ( defined (LIGHTMAP_OFF) && defined(DYNAMICLIGHTMAP_OFF) )
            #define _GLOSSYENV 1
            #include "UnityCG.cginc"
            #include "Lighting.cginc"
            #include "UnityPBSLighting.cginc"
            #include "UnityStandardBRDF.cginc"
            #include "UnityMetaPass.cginc"
            #pragma fragmentoption ARB_precision_hint_fastest
            #pragma multi_compile_shadowcaster
            #pragma multi_compile LIGHTMAP_OFF LIGHTMAP_ON
            #pragma multi_compile DIRLIGHTMAP_OFF DIRLIGHTMAP_COMBINED DIRLIGHTMAP_SEPARATE
            #pragma multi_compile DYNAMICLIGHTMAP_OFF DYNAMICLIGHTMAP_ON
            #pragma multi_compile_fog
            #pragma exclude_renderers gles3 metal d3d11_9x xbox360 xboxone ps3 ps4 psp2 
            #pragma target 3.0
            uniform sampler2D _CameraDepthTexture;
            uniform fixed4 _Color;
            uniform fixed _Specular;
            uniform fixed _Gloss;
            uniform fixed _Depth;
            uniform float _WaterDistance;
            struct VertexInput {
                float4 vertex : POSITION;
                float2 texcoord1 : TEXCOORD1;
                float2 texcoord2 : TEXCOORD2;
            };
            struct VertexOutput {
                float4 pos : SV_POSITION;
                float2 uv1 : TEXCOORD0;
                float2 uv2 : TEXCOORD1;
                float4 posWorld : TEXCOORD2;
                float4 projPos : TEXCOORD3;
            };
            VertexOutput vert (VertexInput v) {
                VertexOutput o = (VertexOutput)0;
                o.uv1 = v.texcoord1;
                o.uv2 = v.texcoord2;
                o.posWorld = mul(unity_ObjectToWorld, v.vertex);
                o.pos = UnityMetaVertexPosition(v.vertex, v.texcoord1.xy, v.texcoord2.xy, unity_LightmapST, unity_DynamicLightmapST );
                o.projPos = ComputeScreenPos (o.pos);
                COMPUTE_EYEDEPTH(o.projPos.z);
                return o;
            }
            float4 frag(VertexOutput i) : SV_Target {
                float3 viewDirection = normalize(_WorldSpaceCameraPos.xyz - i.posWorld.xyz);
                float sceneZ = max(0,LinearEyeDepth (UNITY_SAMPLE_DEPTH(tex2Dproj(_CameraDepthTexture, UNITY_PROJ_COORD(i.projPos)))) - _ProjectionParams.g);
                float partZ = max(0,i.projPos.z - _ProjectionParams.g);
                UnityMetaInput o;
                UNITY_INITIALIZE_OUTPUT( UnityMetaInput, o );
                
                o.Emission = 0;
                
                fixed node_4138 = saturate((sceneZ-partZ)/_Depth);
                fixed waterFog = node_4138;
                float3 diffColor = (_Color.rgb*(1.0 - waterFog));
                float distanceMult = (1.0 - saturate(((partZ/_WaterDistance)*1.492537+-0.4925373)));
                fixed node_3010 = distanceMult;
                float node_3669 = (_Specular*node_3010);
                float3 specColor = float3(node_3669,node_3669,node_3669);
                float specularMonochrome = max(max(specColor.r, specColor.g),specColor.b);
                diffColor *= (1.0-specularMonochrome);
                float roughness = 1.0 - _Gloss;
                o.Albedo = diffColor + specColor * roughness * roughness * 0.5;
                
                return UnityMetaFragment( o );
            }
            ENDCG
        }
    }
    FallBack "Diffuse"
    CustomEditor "ShaderForgeMaterialInspector"
}
